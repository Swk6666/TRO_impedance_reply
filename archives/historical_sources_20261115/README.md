# Historical experiment source archive — 2026-11-15

This archive preserves all 241 historical source files found under the ignored
`outputs/` directory before the user-requested removal of the local checkout and
its experiment data. The files retain their original repository-relative paths.

The source checkout was at commit `a365866fa5dbe294ecd2b8a5344b43e227b0b445`.
Its `main` branch matched the remote before archival. Of the archived files,
75 files (27 distinct Git blob contents) were absent from that commit's complete
history. The other 166 are included to keep the source snapshots together.
Total uncompressed source size: 1,605,482 bytes.

- `sources.tar.gz`: historical Python source snapshots and the comparison script.
- `manifest.json`: original paths, sizes, file modes, modification times, SHA256
  hashes, Git blob hashes, and whether each file was already in the base history.

The archive includes the existing grinding/E snapshots as historical evidence;
it does not resume the deferred experiment. It preserves source bytes without
changing current runtime code. Numerical experiment outputs, videos, virtual
environments, and build caches are not included. Restoring this archive does
not restore the deleted experimental data or guarantee a complete historical
runtime environment.

## Verify and restore

From a fresh checkout, verify the archive and every member:

```bash
python3 - <<'PYVERIFY'
import hashlib, json, tarfile
from pathlib import Path
root = Path('archives/historical_sources_20261115')
manifest = json.loads((root / 'manifest.json').read_text())
archive = root / manifest['archive_file']
assert hashlib.sha256(archive.read_bytes()).hexdigest() == manifest['archive_sha256']
with tarfile.open(archive, 'r:gz') as tar:
    members = {member.name: member for member in tar.getmembers()}
    assert len(members) == manifest['source_file_count']
    assert set(members) == {item['original_path'] for item in manifest['files']}
    for item in manifest['files']:
        member = members[item['original_path']]
        assert member.isfile()
        data = tar.extractfile(member).read()
        assert len(data) == item['size_bytes']
        assert hashlib.sha256(data).hexdigest() == item['sha256']
print('Verified all 241 source files')
PYVERIFY

# Run in a fresh checkout without an existing outputs/ directory.
tar -xzf archives/historical_sources_20261115/sources.tar.gz
```
