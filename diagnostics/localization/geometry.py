"""Offline map/scan replay. Synthetic evidence never establishes hardware alignment."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import statistics


def integrate(pose, linear, angular, dt):
    x,y,yaw=pose
    if abs(angular)<1e-9:
        return [x+linear*dt*math.cos(yaw),y+linear*dt*math.sin(yaw),yaw]
    end=yaw+angular*dt
    return [x+linear/angular*(math.sin(end)-math.sin(yaw)),
            y-linear/angular*(math.cos(end)-math.cos(yaw)),end]


class MapRaster:
    def __init__(self, path, map_id, revision):
        self.path=Path(path)
        # Nav2's checked-in map YAML uses scalar fields and a numeric origin.
        import yaml
        metadata=yaml.safe_load(self.path.read_text())
        self.resolution=float(metadata['resolution'])
        self.origin=metadata['origin']
        self.negate=int(metadata['negate'])
        self.occupied=float(metadata['occupied_thresh'])
        self.free=float(metadata['free_thresh'])
        if self.resolution<=0 or not 0<=self.free<self.occupied<=1:
            raise ValueError('invalid map thresholds/resolution')
        data=(self.path.parent/metadata['image']).read_bytes()
        tokens=[];offset=0
        while len(tokens)<4:
            match=re.match(rb'\s*(?:#[^\n]*\n\s*)*([^\s#]+)',data[offset:])
            if not match: raise ValueError('invalid PGM header')
            tokens.append(match[1]);offset+=match.end()
        if tokens[0]!=b'P5' or tokens[3]!=b'255':
            raise ValueError('requires 8-bit binary PGM')
        if data[offset:offset+2]==b'\r\n': offset+=2
        else: offset+=1
        self.width,self.height=map(int,tokens[1:3])
        self.pixels=bytearray(data[offset:])
        if len(self.pixels)!=self.width*self.height: raise ValueError('invalid PGM raster size')
        self.identity={'map_id':map_id,'revision':revision,'yaml':str(self.path),
                       'yaml_sha256':hashlib.sha256(self.path.read_bytes()).hexdigest(),
                       'image_sha256':hashlib.sha256(data).hexdigest(),'metadata':metadata}

    def local(self,x,y):
        x-=self.origin[0];y-=self.origin[1];yaw=self.origin[2]
        return ((x*math.cos(yaw)+y*math.sin(yaw))/self.resolution,
                (-x*math.sin(yaw)+y*math.cos(yaw))/self.resolution)

    def grid_cell(self,x,y):
        if not (0<=x<self.width and 0<=y<self.height): return 'out_of_map'
        pixel=self.pixels[(self.height-1-y)*self.width+x]
        occupancy=pixel/255 if self.negate else 1-pixel/255
        return 'occupied' if occupancy>self.occupied else ('free' if occupancy<self.free else 'unknown')

    def cell(self,x,y):
        x,y=self.local(x,y)
        return self.grid_cell(math.floor(x),math.floor(y))

    def raycast(self,xy,angle,max_range):
        x,y=self.local(*xy);angle-=self.origin[2]
        dx,dy=math.cos(angle)/self.resolution,math.sin(angle)/self.resolution
        ix,iy=math.floor(x),math.floor(y);distance=0.;axis=None
        while distance<=max_range:
            state=self.grid_cell(ix,iy)
            if state!='free': return {'classification':state,'distance':distance,'axis':axis}
            tx=((ix+1 if dx>0 else ix)-x)/dx if abs(dx)>1e-12 else math.inf
            ty=((iy+1 if dy>0 else iy)-y)/dy if abs(dy)>1e-12 else math.inf
            if tx<ty: distance=tx;ix+=1 if dx>0 else -1;axis='x'
            else: distance=ty;iy+=1 if dy>0 else -1;axis='y'
        return {'classification':'no_surface','distance':max_range,'axis':None}

    def compare(self,xy,angle,measured):
        hit=self.raycast(xy,angle,math.hypot(self.width,self.height)*self.resolution)
        hit['residual']=measured-hit['distance'] if hit['classification']=='occupied' else None
        if hit['residual'] is not None and hit['residual']<-.15: hit['classification']='occluded'
        return hit


def fit_line(points):
    if len(points)<8:
        return None
    x=statistics.mean(p[0] for p in points);y=statistics.mean(p[1] for p in points)
    xx=sum((p[0]-x)**2 for p in points);yy=sum((p[1]-y)**2 for p in points)
    xy=sum((p[0]-x)*(p[1]-y) for p in points)
    angle=.5*math.atan2(2*xy,xx-yy)
    normal=(-math.sin(angle),math.cos(angle))
    offset=normal[0]*x+normal[1]*y
    errors=[normal[0]*p[0]+normal[1]*p[1]-offset for p in points]
    along=[math.cos(angle)*p[0]+math.sin(angle)*p[1] for p in points]
    return {'angle':angle,'normal':normal,'offset':offset,
            'span_m':max(along)-min(along),'rms_m':math.sqrt(statistics.mean(e*e for e in errors)),
            'support':len(points)}


def intersection(a,b):
    ax,ay=a['normal'];bx,by=b['normal'];det=ax*by-ay*bx
    if abs(det)<math.sin(math.radians(20)):
        return None
    return ((a['offset']*by-ay*b['offset'])/det,
            (ax*b['offset']-a['offset']*bx)/det)


def occupied_segment(raster,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    steps=max(1,math.ceil(math.hypot(dx,dy)/raster.resolution*2))
    for i in range(steps+1):
        x,y=raster.local(a[0]+dx*i/steps,a[1]+dy*i/steps)
        ix,iy=math.floor(x),math.floor(y)
        if not any(raster.grid_cell(ix+ox,iy+oy)=='occupied'
                   for ox in (-1,0,1) for oy in (-1,0,1)):
            return False
    return True


def structural_geometry(hits,raster):
    resolution=raster.resolution
    # Map endpoints select candidate walls; measured endpoints must independently fit them.
    remaining=[h for h in hits if h['classification']=='occupied']
    walls=[];tolerance=max(.02,resolution*1.5)
    for _ in range(2):
        best=[]
        candidates=remaining[::max(1,len(remaining)//24)]
        for i,a in enumerate(candidates):
            for b in candidates[i+1:]:
                dx=b['map_endpoint'][0]-a['map_endpoint'][0]
                dy=b['map_endpoint'][1]-a['map_endpoint'][1]
                length=math.hypot(dx,dy)
                if length<.4 or not occupied_segment(raster,a['map_endpoint'],b['map_endpoint']): continue
                selected=[h for h in remaining if abs(
                    dy*(h['map_endpoint'][0]-a['map_endpoint'][0])-
                    dx*(h['map_endpoint'][1]-a['map_endpoint'][1]))/length<=tolerance]
                if len(selected)>len(best): best=selected
        mapped=fit_line([h['map_endpoint'] for h in best])
        measured=fit_line([h['scan_endpoint'] for h in best])
        if not mapped or not measured or mapped['span_m']<.4 or mapped['rms_m']>tolerance:
            break
        angle=abs(math.remainder(measured['angle']-mapped['angle'],math.pi))*180/math.pi
        offsets=[mapped['normal'][0]*h['scan_endpoint'][0]+
                 mapped['normal'][1]*h['scan_endpoint'][1]-mapped['offset'] for h in best]
        residuals=sorted(abs(h['residual']) for h in best)
        walls.append({'map':mapped,'scan':measured,'normal_angle_error_deg':angle,
                      'median_range_residual_m':statistics.median(residuals),
                      'p90_range_residual_m':residuals[math.ceil(.9*len(residuals))-1],
                      'normal_offset_m':abs(statistics.mean(offsets))})
        remaining=[h for h in remaining if h not in best]
    result={'walls':walls,'unassigned_occupied_count':len(remaining),'corner_error_m':None,'identified':False,'pass':False}
    if len(walls)!=2: return result
    mapped_corner=intersection(*(w['map'] for w in walls))
    scan_corner=intersection(*(w['scan'] for w in walls))
    if mapped_corner is None or scan_corner is None: return result
    result.update(identified=True,map_corner=mapped_corner,scan_corner=scan_corner,
                  corner_error_m=math.dist(mapped_corner,scan_corner))
    result['pass']=(result['corner_error_m']<=.15 and all(
        w['normal_angle_error_deg']<=3 and w['normal_offset_m']<=.10 and
        w['median_range_residual_m']<=.10 and w['p90_range_residual_m']<=.15 and
        w['scan']['span_m']>=.4 and w['scan']['rms_m']<=.05 for w in walls))
    return result


def analyze(raster,scan,supplied_pose,amcl_pose,laser=(0,0,0)):
    overlays={}
    for name,pose in [('supplied',supplied_pose),('amcl',amcl_pose)]:
        x,y,yaw=pose
        xy=(x+math.cos(yaw)*laser[0]-math.sin(yaw)*laser[1],
            y+math.sin(yaw)*laser[0]+math.cos(yaw)*laser[1])
        hits=[];excluded=0
        for i,value in enumerate(scan['ranges']):
            if (value is None or not math.isfinite(value) or value<=0 or
                    value<scan.get('range_min',0.) or value>=scan.get('range_max',math.inf)):
                excluded+=1;continue
            beam=yaw+laser[2]+scan['angle_min']+i*scan['angle_increment']
            hit=raster.compare(xy,beam,value)
            hit['map_endpoint']=[xy[0]+math.cos(beam)*hit['distance'],xy[1]+math.sin(beam)*hit['distance']]
            hit['scan_endpoint']=[xy[0]+math.cos(beam)*value,xy[1]+math.sin(beam)*value]
            hits.append(hit)
        residuals=sorted(abs(h['residual']) for h in hits if h['classification']=='occupied')
        geometry=structural_geometry(hits,raster)
        overlays[name]={'pose':list(pose),'hits':hits,'excluded_ranges_count':excluded,'median':statistics.median(residuals) if residuals else None,
            'p90':residuals[max(0,math.ceil(.9*len(residuals))-1)] if residuals else None,
            'second_surface':geometry['identified'],'geometry':geometry}
    displacement=math.hypot(amcl_pose[0]-supplied_pose[0],amcl_pose[1]-supplied_pose[1])
    angle=abs(math.remainder(amcl_pose[2]-supplied_pose[2],2*math.pi))*180/math.pi
    a=overlays['amcl']
    verdict='inconclusive'
    if a['geometry']['identified'] and a['median'] is not None:
        verdict='pass' if a['median']<=.10 and a['p90']<=.15 and a['geometry']['pass'] else 'fail'
    if displacement > .10 or angle > 3:
        verdict='fail'
    return {'map':raster.identity,'overlays':overlays,'displacement_m':displacement,
            'angle_difference_deg':angle,'verdict':verdict,'physical_acceptance':False}


def compose(a,b):
    return [a[0]+math.cos(a[2])*b[0]-math.sin(a[2])*b[1],
            a[1]+math.sin(a[2])*b[0]+math.cos(a[2])*b[1],
            math.remainder(a[2]+b[2],2*math.pi)]


def check_evidence(evidence):
    """Transforms are explicit planar readbacks at the same scan timestamp.

    evidence.transforms: [{parent, child, stamp_ns, pose: [x,y,yaw]}, ...].
    map->base_link is an independent lookup, compared numerically to the chain.
    Only base_link->laser_frame may be explicitly static with stamp_ns=0.
    Edge names without timed values are insufficient evidence.
    """
    now=evidence['now_ns']
    ages={key:(now-evidence[key])/1e9 for key in ('scan_stamp_ns','odom_stamp_ns','amcl_stamp_ns') if key in evidence}
    required={('map','odom'),('odom','base_link'),('base_link','laser_frame'),('map','base_link')}
    transforms={}
    for transform in evidence.get('transforms',[]):
        edge=(transform.get('parent'),transform.get('child'))
        pose=transform.get('pose',[])
        if len(pose)==3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in pose):
            transforms[edge]=transform
    valid=required<=transforms.keys()
    error=None
    if valid:
        stamp=evidence.get('scan_stamp_ns')
        valid=stamp is not None and all(
            transforms[edge].get('stamp_ns')==stamp or
            (edge==('base_link','laser_frame') and transforms[edge].get('static') is True and
             transforms[edge].get('stamp_ns')==0) for edge in required)
        if valid:
            predicted=compose(transforms['map','odom']['pose'],transforms['odom','base_link']['pose'])
            observed=transforms['map','base_link']['pose']
            error={'position_m':math.dist(predicted[:2],observed[:2]),
                   'heading_deg':abs(math.remainder(predicted[2]-observed[2],2*math.pi))*180/math.pi}
            valid=error['position_m']<=.01 and error['heading_deg']<=.3
    poses=evidence.get('poses',[])
    spread=max((math.hypot(a[0]-b[0],a[1]-b[1]) for a in poses for b in poses),default=None)
    # Use the existing scan/odometry watchdog freshness limit.
    timing=(len(ages)==3 and all(0<=age<=.5 for age in ages.values()) and
            evidence['amcl_stamp_ns']==evidence['scan_stamp_ns'])
    return {'ages_s':ages,'timing_ok':timing,'tf_chain_ok':valid and timing,
            'tf_composition_error':error,'spread_m':spread,
            'note':'Numerical TF consistency does not establish physical alignment'}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('capture',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();capture=json.loads(args.capture.read_text())
    identity=capture['map'];raster=MapRaster(identity['yaml'],identity['map_id'],identity['revision'])
    missing=list(capture.get('missing',[]))
    if any(identity.get(key)!=raster.identity[key] for key in ('yaml_sha256','image_sha256')):
        missing.append('map_identity_mismatch')
    if capture.get('complete') is False or missing:
        args.output.write_text(json.dumps({'verdict':'inconclusive','missing':missing,'physical_acceptance':False},indent=2)+'\n')
        return
    result=analyze(raster,capture['scan'],capture['supplied_pose'],capture['amcl_pose'],capture['laser_transform'])
    result['capture_identity']=capture.get('identity',{})
    result['evidence']=check_evidence(capture.get('evidence',{'now_ns':0}))
    if not result['evidence']['timing_ok'] or not result['evidence']['tf_chain_ok']:
        result['verdict']='inconclusive'
    args.output.write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__': main()
