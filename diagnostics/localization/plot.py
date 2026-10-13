"""Render first-surface comparisons from geometry.py JSON output."""
import argparse
import json
import math
from pathlib import Path


def render(report, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.transforms import Affine2D
    from geometry import MapRaster
    raster=MapRaster(report['map']['yaml'],report['map']['map_id'],report['map']['revision'])
    points=[h[k] for overlay in report['overlays'].values() for h in overlay['hits']
            for k in ('map_endpoint','scan_endpoint')]
    xs,ys=zip(*points)
    span=max(max(xs)-min(xs),max(ys)-min(ys))+.6
    cx,cy=(max(xs)+min(xs))/2,(max(ys)+min(ys))/2
    bounds=(cx-span/2,cx+span/2,cy-span/2,cy+span/2)
    local=[raster.local(x,y) for x in bounds[:2] for y in bounds[2:]]
    ix0=max(0,math.floor(min(p[0] for p in local)));ix1=min(raster.width,math.ceil(max(p[0] for p in local)))
    iy0=max(0,math.floor(min(p[1] for p in local)));iy1=min(raster.height,math.ceil(max(p[1] for p in local)))
    pixels=np.frombuffer(raster.pixels,dtype=np.uint8).reshape(raster.height,raster.width)
    crop=pixels[raster.height-iy1:raster.height-iy0,ix0:ix1]/255.
    occupancy=crop if raster.negate else 1-crop
    semantic=np.where(occupancy>raster.occupied,0.,np.where(occupancy<raster.free,1.,.6))
    fig, axes = plt.subplots(1, 2, figsize=(12, 6), constrained_layout=True)
    for ax, name in zip(axes, ('supplied', 'amcl')):
        transform=Affine2D().rotate(raster.origin[2]).translate(*raster.origin[:2])+ax.transData
        ax.imshow(semantic,cmap='gray',vmin=0,vmax=1,origin='upper',interpolation='nearest',
                  extent=(ix0*raster.resolution,ix1*raster.resolution,iy0*raster.resolution,iy1*raster.resolution),transform=transform)
        overlay = report['overlays'][name]
        hits = overlay['hits']
        for label, key, color in [('Mapped first surface', 'map_endpoint', 'tab:purple'),
                                  ('Observed scan', 'scan_endpoint', 'tab:blue')]:
            points = [h[key] for h in hits if h['classification'] == 'occupied']
            if points:
                ax.scatter(*zip(*points), s=9, c=color, label=label)
        for state, color in [('occluded', 'tab:orange'), ('unknown', 'gray'), ('out_of_map', 'tab:red')]:
            points = [h['scan_endpoint'] for h in hits if h['classification'] == state]
            if points:
                ax.scatter(*zip(*points), s=9, c=color, label=state)
        median='n/a' if overlay['median'] is None else f'{overlay["median"]:.3f}'
        ax.set_title(f'{name.capitalize()} pose | median={median} m')
        ax.set(xlabel='Map X (m)', ylabel='Map Y (m)', aspect='equal')
        ax.set_xlim(bounds[:2]);ax.set_ylim(bounds[2:])
        ax.ticklabel_format(useOffset=False)
        if 'pose' in overlay:
            x,y,yaw=overlay['pose'];ax.plot(x,y,'r+',markersize=8)
            ax.arrow(x,y,.25*math.cos(yaw),.25*math.sin(yaw),color='red',width=.008)
        ax.grid(alpha=.2)
        if ax.has_data(): ax.legend(fontsize=8)
    fig.suptitle(f'{report["map"]["map_id"]} / {report["map"]["revision"][:19]}…: {report["verdict"]}\n'
                 'Geometry diagnostic; physical acceptance requires independent checks')
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    render(json.loads(args.report.read_text()), args.output)


if __name__ == '__main__':
    main()
