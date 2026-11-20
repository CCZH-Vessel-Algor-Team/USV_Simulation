# CAD-derived visual meshes

These assets were generated from CAD archives supplied by the workspace user:

- `ship-mast-light-1.snapshot.1.zip`, member `MastLight/MastLight.stp`.
- `marine-navigation-light-all-round-1.snapshot.2.zip`, member `Marine navigation light.stp`.

**No licence or author attribution was identified in the archives. These meshes
are not relicensed under the source code's Apache-2.0 licence. Confirm the CAD
owner's distribution rights before publishing them.** Original archives are not
copied into this repository. SHA-256 provenance, tessellation settings, coordinate
transforms, bounds and triangle counts are recorded in `cad_assets.json`.

All STL coordinates are already in **metres**; do not apply another 0.001 scale.

| File | Purpose |
| --- | --- |
| `mast_base.stl` | Fixed bottom bracket and lower shaft, relative to the mounting datum. |
| `mast_body.stl` | Moving mast and fittings above the fixed bracket, excluding all eleven replaced decorative lamps; relative to the virtual hinge. |
| `mast_lens.stl` | Original top lens only, centred at its own geometry origin for independent white-light control. |
| `navigation_lamp_housing.stl` | One closed lamp's housing, cap, latch and internal fittings. |
| `navigation_lamp_lens.stl` | The closed lamp's lens, relative to the same bottom mounting datum. |

The native mesh is approximately 0.597869 m tall (default runtime XYZ scale 2,
total height 1.195738 m). Its pole axis is the mounting datum's
X/Y origin; source CAD X is shifted by -63.876742 mm. The virtual transverse
folding axis is 0.121037 m above the mounting datum. This geometric split preserves
the upright CAD exterior but **does not represent a mechanically verified hinge**.
The fixed lower bracket is not rotated with the moving shaft.

The original lamps on the two paired crossbars (24 solids plus four open-shell
lenses) are excluded from the moving mesh. The source mounting pads/beams are
retained. Their four mounting datums, relative to the moving hinge, are recorded
as `navigation_light_mounts_from_hinge_m`; new controlled lamps are placed there.
Legacy `front/rear` IDs denote the upper/lower tiers. The other seven decorative
lamps (36 solids and three open shells) are also removed, retaining supports.
`extra_light_mounts_from_hinge_m` records their datums and fixture size factors;
`extra_light_order` defines their appended state-array order. Source filtering
is checked before export and by tests.

The navigation-light source includes two demonstration poses, closed at CAD Y=0
and open at Y=205.76 mm. Only the closed lamp is exported: approximately 0.152 m
wide and 0.200034 m tall. Its bottom is Z=0. The display example with the open cap
is deliberately excluded.

The STEP exports do not contain the rendered carbon-fibre/glass appearance seen
in the JPG previews. Runtime materials approximate a dark housing and emissive
coloured lenses; they do not reproduce optical refraction, regulatory sectors,
or the CAD preview's exact textures. All eleven replacement lamps have separate
red/green controls; the original top lens retains its existing white-lamp API.

## Regeneration (offline only)

With Python 3.10 and `cadquery-ocp==7.7.2` available in a separate conversion
environment, run from the package directory:

```bash
python tools/convert_cad_assets.py \
  --mast-archive /path/to/ship-mast-light-1.snapshot.1.zip \
  --lamp-archive /path/to/marine-navigation-light-all-round-1.snapshot.2.zip \
  --output description/meshes
```

The script checks the input units and component selection before export. It
retains CAD surfaces except the eleven explicitly replaced old lamps, splits triangles at the virtual hinge, and writes
three mast meshes, two lamp meshes and their profile. No CAD software or OCP
Python module is required to build or run the simulation.
