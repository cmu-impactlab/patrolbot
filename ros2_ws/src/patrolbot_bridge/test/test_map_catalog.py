import json
from pathlib import Path
import shutil

from patrolbot_bridge.map_catalog import ActiveMapStore, MapCatalog
from patrolbot_bridge.map_transaction import MapTransaction
import pytest

MAPS = Path(__file__).resolve().parents[2] / 'patrolbot_navigation/maps'


def test_catalog_and_bounds():
    catalog = MapCatalog(MAPS)
    for key, entry in catalog.maps.items():
        assert catalog.require(key, entry['revision']) == entry
        x, y, _ = entry['origin']
        assert catalog.contains(entry, x, y)
        assert not catalog.contains(entry, x - 0.01, y)
        assert not catalog.contains(entry, x + entry['width'] * entry['resolution'], y)
        assert not catalog.contains(entry, float('nan'), y)
        with pytest.raises(ValueError):
            catalog.require(key, 'old')


def test_changed_map_fails_catalog(tmp_path):
    shutil.copytree(MAPS, tmp_path / 'maps')
    p = tmp_path / 'maps/cmuq_1st_floor.yaml'
    p.write_text(p.read_text().replace('0.050', '0.051'))
    with pytest.raises(ValueError, match='revision mismatch'):
        MapCatalog(p.parent)


def test_persistence_first_install_restart_and_corruption(tmp_path):
    catalog = MapCatalog(MAPS)
    store = ActiveMapStore(tmp_path, catalog)
    assert store.load()['map_id'] == 'cmuq-floor2'
    first = catalog.maps['cmuq-floor1']
    store.save(first['map_id'], first['revision'])
    assert ActiveMapStore(tmp_path, catalog).load() == first
    store.path.write_text('{broken')
    with pytest.raises(ValueError):
        store.load()
    store.path.write_text(json.dumps({
        'schema_version': 1, 'map_id': 'cmuq-floor1', 'revision': 'old',
    }))
    with pytest.raises(ValueError):
        store.load()
    with pytest.raises(ValueError, match='unavailable'):
        ActiveMapStore(tmp_path / 'missing-volume', catalog).load()


def begin(transaction, **overrides):
    entry = transaction.catalog.maps['cmuq-floor1']
    values = {
        'request_id': 'bd98f746-f930-49e9-8b8f-17395cd84a65',
        'map_id': entry['map_id'],
        'revision': entry['revision'],
        'pose': (1.0, 1.0, 0.0),
        'authorized': True,
        'origin': 'origin',
        'session': 1,
        'telemetry_age': 0.1,
        'linear': 0.0,
        'angular': 0.0,
        'navigation': False,
        'undock': False,
        'hardware_busy': False,
    }
    values.update(overrides)
    return transaction.begin(**values)


@pytest.mark.parametrize(
    'override',
    [
        {'authorized': False},
        {'telemetry_age': 0.6},
        {'telemetry_age': -1},
        {'linear': 0.006},
        {'angular': 0.011},
        {'linear': float('nan')},
        {'navigation': True},
        {'undock': True},
        {'hardware_busy': True},
        {'origin': None},
        {'revision': 'old'},
        {'map_id': '../../tmp'},
        {'pose': (-1.0, 0.0, 0.0)},
    ],
)
def test_refusals(override):
    transaction = MapTransaction(MapCatalog(MAPS))
    with pytest.raises(ValueError):
        begin(transaction, **override)
    assert transaction.stage == 'idle'


@pytest.mark.parametrize('failed_stage', MapTransaction.STAGES[:-1])
def test_failure_retains_hold_and_ignores_late_ack(failed_stage):
    transaction = MapTransaction(MapCatalog(MAPS))
    token = begin(transaction)
    while transaction.stage != failed_stage:
        assert transaction.advance(token, transaction.stage)
    previous = transaction.active
    transaction.fail('timeout/disconnect/cancel')
    assert transaction.hold
    assert not transaction.advance(token, failed_stage)
    assert transaction.active == previous
    with pytest.raises(ValueError):
        begin(transaction)


def test_ordering_concurrency_activation_and_single_seed():
    transaction = MapTransaction(MapCatalog(MAPS))
    token = begin(transaction)
    with pytest.raises(ValueError):
        begin(transaction, request_id='986e4515-5de8-4332-bb9a-7de5773ee93d')
    assert not transaction.advance(None, 'hold')
    assert not transaction.advance(token, 'verify-map')
    while transaction.stage != 'seed':
        assert transaction.hold
        transaction.advance(token, transaction.stage)
    assert transaction.activated and transaction.active['map_id'] == 'cmuq-floor1'
    assert transaction.seeded(token, 123)
    assert not transaction.seeded(token, 124)
    while transaction.stage != 'ready':
        assert transaction.hold
        transaction.advance(token, transaction.stage)
    assert not transaction.hold
