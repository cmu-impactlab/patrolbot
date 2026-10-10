from pathlib import Path
import tempfile,hashlib,stat
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rollout_files import install, restore, digest

def fixture(path):
 root=path/'root';payload=path/'payload';backup=path/'backup'
 for p in (root/'docker',root/'src',payload/'src'):p.mkdir(parents=True)
 (root/'docker/.env').write_text('PATROLBOT_IMAGE=old\nUNCHANGED=opaque\n');(root/'docker/.env').chmod(0o600)
 changes=[]
 for n in ['a.py','b.py','launch.py']:
  r='src/'+n;(root/r).write_text('before '+n);(payload/r).write_text('after '+n);(root/r).chmod(0o750)
  changes.append(dict(path=r,before=digest(root/r),after=digest(payload/r)))
 return root,payload,backup,changes

def test_transaction_cases():
    for case in ['success_restore','verification_failure','stale_source','bad_payload','corrupt_backup']:
     with tempfile.TemporaryDirectory() as d:
      root,payload,backup,changes=fixture(Path(d))
      before={str(p.relative_to(root)):p.read_bytes() for p in root.rglob('*') if p.is_file()}
      if case=='stale_source':changes[0]['before']='bad'
      if case=='bad_payload':changes[0]['after']='bad'
      def verify():
       if case=='verification_failure':raise RuntimeError('simulated failed post-write check')
      try:
       install(root,payload,backup,changes,'old','new',verify)
       assert case not in ('verification_failure','stale_source','bad_payload')
      except (AssertionError,RuntimeError):
       assert case in ('verification_failure','stale_source','bad_payload')
      if case=='success_restore':
       assert 'PATROLBOT_IMAGE=new' in (root/'docker/.env').read_text()
       assert all(digest(root/c['path'])==c['after'] for c in changes)
       restore(root,backup)
      if case=='corrupt_backup':
       current={str(p.relative_to(root)):p.read_bytes() for p in root.rglob('*') if p.is_file()}
       (backup/changes[0]['path']).write_text('corrupted')
       try:restore(root,backup);raise RuntimeError('corrupt snapshot accepted')
       except AssertionError:pass
       assert current=={str(p.relative_to(root)):p.read_bytes() for p in root.rglob('*') if p.is_file()}
      else:
       assert before=={str(p.relative_to(root)):p.read_bytes() for p in root.rglob('*') if p.is_file()}
      assert stat.S_IMODE((root/'docker/.env').stat().st_mode)==0o600
      assert all(stat.S_IMODE((root/c['path']).stat().st_mode)==0o750 for c in changes)
      assert not list(root.rglob('*.patrolbot-stage'))
     print('PASS',case)

if __name__ == '__main__':
    test_transaction_cases()
