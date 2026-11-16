# Marswalk: Jezero EVA Planner

NASA Space Apps Challenge 2026 entry for **Interplanetary Survival Guide: Martian Map**.

Marswalk is a 3D, browser-based planner for astronaut walks (EVAs) on real Mars terrain at Jezero Crater. It does four things:
- finds the lowest-energy route under a slope limit
- checks the suit time limit and crew readiness
- replays real solar storms from NASA DONKI that were predicted to hit Mars
- plans an abort route back to the habitat

**Run locally:** `python3 -m http.server 8765`, then open http://localhost:8765. It has to be served over HTTP; opening the file directly won't work.

**Data:**
- NASA/JPL/USGS Mars 2020 TRN CTX elevation model and orthomosaic of Jezero
- HRSC-MOLA blended 200 m elevation model (NASA + ESA, USGS) and Viking color mosaic (NASA Mars Trek) for the surrounding region
- NASA CCMC DONKI
- Mars 2020 Perseverance traverse (NASA/JPL MMGIS)
- NASA/JPL-Caltech Perseverance raw images

Full citations, methods and limits are in the in-app "Data, methods & limits" panel and in `SUBMISSION.md`.

**AI disclosure:** code was written with help from an AI coding assistant and reviewed by the team.

**License:** Apache-2.0. three.js is MIT.
