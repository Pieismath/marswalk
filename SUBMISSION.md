# Space Apps 2026 submission kit: Marswalk (Interplanetary Survival Guide: Martian Map)

Paste these into the Project tab: profile icon → My Team → Project → Edit Details. Edit anything in [brackets]. Rewrite in your own words wherever you like: judges reward your team's thinking.

---

## Project Name
Marswalk: a 3D EVA planner for Jezero Crater

## High-Level Project Summary (paste)
Marswalk is a 3D mission-planning map for the first astronauts to walk on Mars. You fly over real NASA terrain of Jezero Crater, where Perseverance landed, and pick a science target. Marswalk then plans the lowest-energy walking route that stays under a safe slope limit. It checks whether the outbound walk, science time and return walk fit inside the suit's time limit. It flags crew fatigue, and it replays real solar storms from NASA's DONKI database to show whether the crew could reach shelter in time. Current Mars maps are built for orbit-scale science. Marswalk turns NASA data into a clear go/no-go decision at the scale of one astronaut's walk, with safety first.

## Link to Project Demo
[Public link to your 7-slide deck OR ≤30 s video, viewable with no login. Test it in a private window.]

## Link to Final Project
[GitHub Pages URL, e.g. https://YOURNAME.github.io/marswalk/, plus the GitHub repo URL]

## Detailed Project Description (paste)
**What it does**
- **3D terrain:** a CTX stereo elevation model of Jezero Crater (20 m/px, resampled to 60 m) with a 6 m/px CTX orthomosaic draped over it, set inside 240 km of surrounding terrain from the HRSC-MOLA blended elevation model (NASA + ESA) and the Viking color mosaic, with real Mars curvature and a dusty horizon. Rendered in the browser with three.js.
- **Layers** (the challenge asks for at least 3):
  1. Optical imagery
  2. Topography: an elevation color ramp with 50 m contours, and a derived slope-hazard map (red = steeper than the crew's limit, yellow = near the limit)
  3. Space weather: real coronal mass ejections (CMEs) whose NASA WSA-ENLIL model runs predicted a Mars impact, from DONKI, shown on an interactive 2024–2026 timeline (click any CME to replay it)
  4. Rover ground truth: Perseverance's real 45 km drive path (sols 0–2022) and Navcam/Hazcam frames from the landing site
  5. Mars local time: the Mars24 algorithm gives local mean solar time at Jezero, so EVAs that run into darkness get flagged
- **Route planning (sense → decide → act):**
  - *Sense:* slope from the elevation model.
  - *Decide:* an A* search for the minimum-energy path, using the Minetti et al. (2002) walking cost against gradient. Cells over the slope limit are no-go. Uphill and downhill cost differently, so outbound and return are planned separately.
  - *Act:* a GO / CAUTION / NO-GO verdict with the reasons listed.
- **Safety checks:**
  - The suit time limit, with a 20% reserve.
  - Crew readiness: short sleep, hours awake by the end of the EVA, and recent "slam shifts". These come from spaceflight sleep and performance research on how sleep loss and a misaligned body clock degrade astronaut performance.
  - Storm abort: when a CME is predicted to arrive mid-EVA, Marswalk finds where the astronaut is at that moment, plans the fastest route back to the habitat, and says whether they make it.
- **Baseline comparison:** every plan is compared with the naive straight line. In the demo, about 9% of the straight line is steeper than the 15° limit (up to 20°); the planned route detours around those scarps.
- **Honesty about limits:** a built-in "Data, methods & limits" panel explains the 60 m resolution (no boulders), Earth-derived energy and speed models, uncertainty in CME arrival times, and that the readiness check is a screening flag, not a medical tool.

**Why it matters:** surface EVAs will be among the riskiest moments of a Mars mission. Mars has no global magnetic field, communication delays rule out real-time help from Earth, and crews will be fatigued. Marswalk shows how open NASA data can support fast, explainable go/no-go decisions on the surface.

**Tools:** JavaScript, three.js (WebGL), HTML/CSS, Python (NumPy, tifffile, Pillow) for preprocessing. Hosted as a static site.

## Use of Super Intelligence (SI)
[Pick the matching dropdown options. Suggested text:] We used an AI coding assistant (Claude) to help write and debug the JavaScript and Python code and to help draft documentation text. The team chose the challenge, the design and the safety logic, and reviewed all the code. No AI-generated images, data or NASA logos are used; every number shown comes from NASA data and published models.

## NASA Data (paste)
- Mars 2020 Terrain Relative Navigation CTX DTM, Jezero Crater: JEZ_ctx_B_soc_008_DTM_MOLAtopography_DeltaGeoid_20m_Eqc_latTs0_lon0. NASA/JPL-Caltech, USGS Astrogeology. https://planetarymaps.usgs.gov/mosaic/mars2020_trn/CTX/
- Mars 2020 TRN CTX orthomosaic, Jezero Crater, 6 m: JEZ_ctx_B_soc_008_orthoMosaic_6m_Eqc_latTs0_lon0. NASA/JPL-Caltech/MSSS, USGS Astrogeology.
- NASA CCMC DONKI (Space Weather Database Of Notifications, Knowledge, Information): CME analyses and WSA-ENLIL Mars impacts, 2024-01-01 to 2026-11-15. https://ccmc.gsfc.nasa.gov/DONKI/ and https://api.nasa.gov/
- Mars HRSC-MOLA Blended DEM Global 200 m v2 (Mars_HRSC_MOLA_BlendDEM_Global_200mp_v2), NASA MGS MOLA + ESA Mars Express HRSC, USGS Astrogeology. https://planetarymaps.usgs.gov/mosaic/Mars/HRSC_MOLA_Blend/
- Viking MDIM2.1 color mosaic (Mars_Viking_MDIM21_ClrMosaic_global_232m), via NASA Mars Trek tiles. https://trek.nasa.gov/mars/
- Mars 2020 Perseverance traverse and end-of-drive waypoints, sols 0–2022 (45.3 km), NASA/JPL-Caltech MMGIS. https://mars.nasa.gov/mmgis-maps/M20/
- Perseverance raw images, sols 0–4.
- NASA 3D Resources: Astronaut and Mars 2020 Perseverance Rover glTF models. https://github.com/nasa/NASA-3D-Resources NASA/JPL-Caltech. https://mars.nasa.gov/mars2020/multimedia/raw-images/
- Mars Trek (challenge resource, used for site context): https://trek.nasa.gov/mars/
- NASA Human Research Program hazards (radiation, isolation, distance, gravity, hostile environment), which informed the readiness and safety logic: https://www.nasa.gov/hrp/hazards/
- MSL Radiation Assessment Detector surface dose: Hassler, D. M. et al. (2014), "Mars' Surface Radiation Environment Measured with the Mars Science Laboratory's Curiosity Rover," *Science* 343, 1244797.
- Mars24 time algorithm: Allison, M. & McEwen, M. (2000), *Planetary and Space Science* 48, 215–235 (NASA GISS).

## Space Agency Partner Data and Other References (paste)
- Minetti, A. E. et al. (2002), "Energy cost of walking and running at extreme uphill and downhill slopes," *J. Applied Physiology* 93, 1039–1046 (route energy cost).
- Tobler, W. (1993), hiking function (walking speed vs. slope), × 0.7 assumed suit factor.
- three.js (MIT license), https://threejs.org; NumPy, Pillow, tifffile (open source).
- ESA Mars Express HRSC (High Resolution Stereo Camera) terrain, blended with NASA MOLA in the USGS HRSC-MOLA 200 m model: the surrounding 240 km region.

---

## 7-slide deck outline (the title slide counts; stay at 7 or fewer)
1. **Title.** Marswalk: plan a safe walk on Mars. Team name and member names. Challenge: *Interplanetary Survival Guide: Martian Map*. Hero screenshot of the 3D crater.
2. **The problem.** The first Mars EVAs will happen far from help, with roughly 3–22 minute one-way communication delays, no global magnetic field, steep terrain and tired crews. Today's maps are orbit-scale, not walk-scale.
3. **Our solution.** A screenshot with the route, hazard overlay and readout. One line: "Pick a target, and Marswalk plans the safest, lowest-energy walk and gives a go/no-go call."
4. **How NASA data makes it possible.** Five icons with sources: CTX elevation model, CTX imagery, DONKI CMEs, Perseverance images, Mars24 time.
5. **The wow moment: storm abort.** A screenshot of the red storm state: "A real CME (Aug 2026, 1,862 km/s) arrives 3 h into the EVA. The crew is 2.1 h from shelter, so the call is NO-GO."
6. **Does it work?** The planned route against the straight line (9% of the straight line is too steep; the route avoids it all). The limits we know about (60 m grid, Earth-based models, uncertain storm timing) and what we'd add next (HiRISE 1 m terrain, rover traverse data, live radiation feeds).
7. **Future vision.** Run it on the habitat display or a helmet heads-up display; reuse it on the Moon's south pole; train Earth teams. Live demo URL and QR code.

## 30-second video script (alternative to slides; must be ≤30 s)
- 0:00 "We're [team], from Philadelphia."
- 0:02 "The first astronauts on Mars will need to walk safely, far from help."
- 0:05 "Meet Marswalk, the first walk-scale map of Jezero Crater."
- 0:10 Screen recording: pick the delta target, watch the route draw around the red scarps. "Real NASA terrain, lowest-energy route, under a slope limit."
- 0:18 Click "Run storm scenario". "A real solar storm from NASA's DONKI hits mid-EVA. Can they make it back? No-go."
- 0:25 "Next: HiRISE 1 m terrain and the Moon's south pole."

## Deploy (free GitHub Pages, ~5 minutes)
1. Create a **public** repo on github.com (e.g. `marswalk`).
2. Upload everything inside this `mars-eva-planner` folder, including `data/`. The total is about 4 MB.
3. Repo → Settings → Pages → Branch: `main`, folder `/ (root)` → Save. Wait about 1 minute.
4. Open `https://YOURNAME.github.io/marswalk/` in a private window and test it.
5. Optional, recommended by the guide: request a FortiGuard web-filter classification for the URL (https://www.fortiguard.com/faq/wfratingsubmit; company "N/A").

## Final checklist (deadline Sun Nov 15, 11:59 PM EST)
- [ ] Everyone on the team is registered for Philadelphia, not waitlisted, and listed on Members
- [ ] Team Contact Information filled in (About tab, Team Owner)
- [ ] All required fields filled in; one demo link (≤7 slides or ≤30 s)
- [ ] Links work logged out; everything in English; no images of anyone under 18
- [ ] AI images watermarked; no NASA logos in any AI output
- [ ] Both checkboxes ticked → **Submit for Judging** → confirmation banner. Resubmit after every later edit.
