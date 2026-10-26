"""Public model URI conversion contract using real package-layout fixtures."""

import ament_index_python.packages
import pytest

from usv_sim_full.model_description import convert_model_uri_to_package_uri


@pytest.fixture
def model_shares(monkeypatch, tmp_path):
    shares = {
        'wamv_description': tmp_path / 'wamv description',
        'usv_sim_full': tmp_path / 'simulation share',
    }
    for package, relative_dir, model in (
        ('wamv_description', 'models', 'wamv_hull'),
        ('wamv_description', 'models', 'shared_model'),
        ('usv_sim_full', 'description/models', 'radar'),
        ('usv_sim_full', 'description/models', 'shared_model'),
    ):
        (shares[package] / relative_dir / model).mkdir(parents=True)
    (shares['usv_sim_full'] / 'description/models/not_a_model').write_text('file')
    monkeypatch.setattr(
        ament_index_python.packages, 'get_package_share_directory',
        lambda package: str(shares[package]),
    )
    return shares


def test_converts_known_meshes_without_changing_other_uris(model_shares):
    source = '''<robot>
  <mesh filename="model://wamv_hull/mesh/hull.dae"/>
  <mesh filename="model://radar/mesh/radar.dae"/>
  <mesh filename="model://unknown/mesh/unknown.dae"/>
  <mesh filename="model://not_a_model/mesh/file.dae"/>
  <mesh filename="package://other/mesh/file.dae"/>
  <mesh filename="file:///tmp/mesh.dae"/>
  <uri>model://no_subpath</uri>
</robot>'''
    expected = source.replace(
        'model://wamv_hull/', 'package://wamv_description/models/wamv_hull/',
    ).replace(
        'model://radar/', 'package://usv_sim_full/description/models/radar/',
    )
    assert convert_model_uri_to_package_uri(source) == expected


def test_simulation_models_retain_precedence_for_duplicate_names(model_shares):
    assert convert_model_uri_to_package_uri('model://shared_model/mesh/hull.dae') == (
        'package://usv_sim_full/description/models/shared_model/mesh/hull.dae'
    )


@pytest.mark.parametrize('missing', ['wamv_description', 'usv_sim_full', 'both'])
def test_unavailable_packages_leave_their_model_uris_unchanged(
        model_shares, monkeypatch, missing):
    def package_share(package):
        if missing in (package, 'both'):
            raise LookupError(f'{package} is unavailable')
        return str(model_shares[package])

    monkeypatch.setattr(
        ament_index_python.packages, 'get_package_share_directory', package_share,
    )
    source = 'model://wamv_hull/mesh/hull.dae\nmodel://radar/mesh/radar.dae'
    expected = source
    if missing == 'wamv_description':
        expected = expected.replace(
            'model://radar/', 'package://usv_sim_full/description/models/radar/',
        )
    elif missing == 'usv_sim_full':
        expected = expected.replace(
            'model://wamv_hull/', 'package://wamv_description/models/wamv_hull/',
        )
    assert convert_model_uri_to_package_uri(source) == expected
