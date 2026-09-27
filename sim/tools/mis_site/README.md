# The real Michigan International Speedway around the courses

`data/mis-site.json` + `mis-ortho.jpg` + `mis-layers.png` + `mis-class.png`
are what `src/render/sitemesh.js` draws around the autocross, endurance,
skidpad and accel courses: lidar terrain laid with the aerial photo, the
oval's walls and catch fence, grandstands and buildings, and trees. Drawing
only -- physics, cones, timing and every recorded run are untouched (the site
is carried into each course's own frame, not the other way round).

All inputs are public domain: USDA NAIP 2022 (60 cm) and USGS 3DEP lidar
(MI_31County_2016_A16). The course placements come from the team's 2026
FSAE Michigan course maps (Helios vault, `2026 Competition Tracks`).

## Rebuilding (all in one work directory, WORK)

1. **Ortho.** NAIP over the bbox at 0.5 m, from the Microsoft Planetary
   Computer (the `nw` tile; its bottom 330 m is empty, fill from `sw`):

       B=-84.2485,42.0575,-84.2330,42.0790
       curl -o naip_sq.png "https://planetarycomputer.microsoft.com/api/data/v1/item/bbox/$B/2564x4778.png?collection=naip&item=mi_m_4208463_nw_16_060_20220901&assets=image&asset_bidx=image%7C1%2C2%2C3"
       (same with item=mi_m_4208463_sw_16_060_20220901 -> naip_sw.png, paste into the black rows)

2. **Lidar.** Tiles 155202/205/207 and 157202/205/207 from
   `.../LPC/Projects/MI_31County_2016_A16/MI_31Co_Lenawee_2017/LAZ/`, 155210 and
   157210 from `.../MI_31Co_Jackson_2016/LAZ/` (rockyweb.usgs.gov/vdelivery/
   Datasets/Staged/Elevation), into `laz/`. Then `python lidar.py` -> `lidar.npz`
   (1 m ground and surface heights; the tiles only classify ground vs not).
   Needs `laspy[lazrs] pyproj scipy`.

3. **Registration.**
   - `map_mask.png` / `ax_mask.png`: the road linework of each overlay PNG
     (grey, not grid; see `match.py` for the thresholds used).
   - `python match.py map_mask.png 0.3844 end` -- brute-force chamfer search of
     the endurance map over the ortho (every 2 deg, 3 scales). One fit stands
     out; `refine.py` then polishes it continuously (scale 0.996, rot 282.94).
   - The autocross map has no linework; it is pinned by the skidpad figure-8
     both maps draw (`xforms.py`), rotated 180 deg from the endurance map.
   - `python trackfit.py`: each track JSON's centreline onto its map's
     coloured course line (median 0 m, p95 0.5 m).
   - `python xforms.py` composes course frame -> map -> ortho into one
     similarity per course (`site_xforms.json`). `render.py` draws any fit on
     the ortho to check it by eye.

4. **Build.** `python build.py WORK` writes the four data files: surface
   classes (pavement / grass / tree / wall / structure), tree tops, wall
   polylines, the ortho calibrated to the renderer's palette, and the course
   placements (skidpad and accel placed here: see the comments).

## Known limits

- Lidar 2016/17, photo 2022; anything built or repaved since is as it was.
- Trees near a course are generic models at the lidar's tree tops and
  heights; further out the woods are a canopy surface.
- Grandstands and buildings are 2 m blocks at the lidar's heights; walls are
  traced polylines with a catch fence where the lidar saw one.
- The traced course files are ~0.5-0.7 % larger than the ground (the Helios
  trace vs the map's grid); the site is scaled to fit them.
