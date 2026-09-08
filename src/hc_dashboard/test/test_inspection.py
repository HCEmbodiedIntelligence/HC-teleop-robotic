import json
from hc_dashboard.inspection import datasets


def test_catalog_skips_invalid_and_outside_root(tmp_path):
    root = tmp_path / 'datasets'
    root.mkdir()
    for name, value in [('valid', {'schema': 'hc-dataset/v1', 'dataset_id': 'ok'}), ('array', []), ('other', {'schema': 'other'})]:
        folder = root / name
        folder.mkdir()
        (folder / 'manifest.json').write_text(json.dumps(value))
    outside = tmp_path / 'outside'
    outside.mkdir()
    (outside / 'manifest.json').write_text(json.dumps({'schema': 'hc-dataset/v1', 'dataset_id': 'outside'}))
    (root / 'link').symlink_to(outside, target_is_directory=True)
    assert [item['dataset_id'] for item in datasets(root)['items']] == ['ok']


def test_missing_catalog_is_read_only(tmp_path):
    root = tmp_path / 'missing'
    assert datasets(root)['items'] == []
    assert not root.exists()
