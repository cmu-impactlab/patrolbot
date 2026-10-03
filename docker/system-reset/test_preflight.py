import copy
import pytest
from preflight import evaluate


def snapshots():
    base=[dict(received=9.9,link=True,hardware=True,epoch_valid=True,age=.1,
        dock='DOCKED_CONFIRMED',motors=False,undock=False,recovery=True,seed=0,epoch='origin') for _ in range(3)]
    odom=[dict(received=9.9,age=.1,linear=0.,angular=0.,values=[0.,1.]) for _ in range(3)]
    ready=[dict(received=9.9,idle=True,motors=False,dock='DOCKED_CONFIRMED',base_age=.1) for _ in range(2)]
    return base,odom,ready


def test_localization_not_required_before_reset_but_new_seed_barrier_required_after():
    b,o,r=snapshots();assert evaluate(b,o,r,10)['safe']
    assert evaluate(b,o,r,10,post_reset=True)['safe']
    b[-1]['seed']=42
    assert not evaluate(b,o,r,10,post_reset=True)['safe']


@pytest.mark.parametrize('stream,key,value',[
    (0,'motors',True),(0,'undock',True),(0,'link',False),(0,'hardware',False),
    (0,'epoch','different'),(0,'dock','UNKNOWN'),(0,'age',float('nan')),
    (1,'linear',.002),(1,'age',-1),(1,'values',[float('inf')]),
    (2,'idle',False),(2,'base_age',2)])
def test_bad_sample_anywhere_refuses(stream,key,value):
    rows=snapshots();rows[stream][0][key]=value
    assert not evaluate(*rows,10)['safe']


@pytest.mark.parametrize('stream',[0,1,2])
def test_stream_that_stops_mid_capture_refuses(stream):
    rows=snapshots()
    for row in rows[stream]:row['received']=5
    assert not evaluate(*rows,10)['safe']
