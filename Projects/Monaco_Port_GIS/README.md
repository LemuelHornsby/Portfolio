# Port Hercule, Monaco — Maritime GIS Data Package

A Geographic Information System (GIS) data package for the harbour of
Port Hercule, Monaco, produced for ship-simulator development.

## What this is

Nine vector layers, seven raster layers, a tabular berth specification, and
a complete cartographic chart, all in coordinate systems chosen for direct
consumption by simulator engines and web GIS tools. The package combines
real OpenStreetMap features with a synthetic bathymetric grid through six
analytical steps (depth-zone classification, channel extraction, vessel-
clearance modelling, exclusion-zone buffering, berth-capacity assessment,
and cost-surface generation for pathfinding).

## Contents

| Category | Format | CRS | Items |
|---|---|---|---|
| Vector layers | GeoJSON | EPSG:4326 | 9 |
| Vector layers | ESRI Shapefile | EPSG:32632 | 9 |
| Bathymetric raster | GeoTIFF (Float32) | EPSG:32632 | 1 |
| Bathymetric raster | ESRI ASCII grid | EPSG:32632 | 1 |
| Derived rasters (depth zones, clearance, cost) | GeoTIFF | EPSG:32632 | 5 |
| Berth specifications | CSV | — | 1 |
| Cartographic chart | PDF, PNG, SVG | EPSG:32632 / — | 3 |
| Project file | QGIS (.qgz) | — | 1 |

Documentation, the full ten-page report, and the verification summary live
in `06_Documentation/`.

## Study area

Port Hercule sits on Monaco's Mediterranean coast. The harbour basin is
roughly 600 m east-to-west and 400 m north-to-south, bounded by Quai
Antoine 1er, Quai des États-Unis, Quai Rainier III, the Digue Sud
breakwater, and the Contre-jetée. The bounding box used throughout is
7.418° E to 7.435° E and 43.728° N to 43.740° N (WGS 84).

## Coordinate reference systems

Acquisition is performed in WGS 84 (EPSG:4326), analysis in WGS 84 / UTM
Zone 32N (EPSG:32632) for metric distances and areas, and vector exports
are issued in both systems so downstream consumers do not have to
reproject.

## Methodology in brief

| Step | Output |
|---|---|
| Depth-zone classification | 4-class raster + polygons (hazard / small craft / medium / unrestricted) |
| Navigable channel extraction | Channel polygon + approach centreline |
| Vessel-draft clearance | Per-class clearance raster + safe-navigation polygon |
| Exclusion-zone buffering | 10 m quay buffers + 20 m hazard buffers, dissolved |
| Berth-capacity assessment | 10 anchor points sampled against bathymetry; max draft + vessel class |
| Cost-surface pathfinding | Cost raster ready for a least-cost-path solver |

Equations and parameter values are given in the full report
(`06_Documentation/Monaco_Port_GIS_Report.pdf`).

## Software

Open source, no commercial dependencies.

- QGIS 3.44.10 "Solothurn" (LTR)
- GDAL/OGR 3.12.4 (with PROJ)
- Python 3.12.13
- NumPy 2.4.4
- Matplotlib

## Reproducibility

The pipeline runs in five stages: data acquisition, data preparation,
spatial analysis, cartographic production, verification. Each stage takes
deterministic inputs and writes deterministic outputs. A complete cold-
start re-run completes in roughly one hundred seconds on a current
consumer workstation. An automated verification routine opens every
output and confirms its coordinate reference system, feature count, and
raster band statistics; the latest run reports thirty-one of thirty-one
checks passed.

## Data sources

- **OpenStreetMap** features via the Overpass API.
  © OpenStreetMap contributors. Licensed under the
  [Open Database Licence v1.0](https://www.openstreetmap.org/copyright).
- **Bathymetric grid:** synthetic, constrained to a hand-defined
  harbour-basin polygon. **Not survey data and not for navigation.**
  Section 7 of the report describes how to substitute a measured grid
  (e.g. from [EMODnet Bathymetry](https://emodnet.ec.europa.eu/en/bathymetry)
  or the [GEBCO grid](https://www.gebco.net)).

## Caveats

- Bathymetric depths are plausible at basin scale but they are not
  measurements; any quantitative claim about a specific berth depends on
  replacing the grid before operational use.
- The chart's symbology approximates the IHO S-52 convention in sRGB and
  is not formally S-52-compliant. It is a portfolio map, not an ECDIS
  publication.
- The `berth_length` column is a uniform 60 m placeholder. The safe-route
  layer is a straight line; the cost-surface raster is ready for a real
  least-cost-path solver.

## References

Full reference list in the report's bibliography. Standards and datasets
relied upon include IHO S-52 (chart display), IHO S-57 (digital
hydrographic data), ISO 19111:2019 (coordinate referencing), the EPSG
Geodetic Parameter Dataset, OpenStreetMap, EMODnet Digital Bathymetry,
and GEBCO 2024.

## Author

Lemuel Hornsby-Odoi · NaviSense Marine Solutions · 2026
