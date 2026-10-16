# Three.js browser dependencies

Vendored Three.js **0.160.0** under the MIT license (see `LICENSE`).
Source project: https://github.com/mrdoob/three.js/tree/r160

Files were obtained from the published `three@0.160.0` npm package:

- `build/three.module.js`
- `examples/jsm/controls/OrbitControls.js`
- `examples/jsm/loaders/STLLoader.js`

The two addon imports of `three` are rewritten to `./three.module.js`.
The browser loads all files from this server; no CDN or internet access is needed.
These modules are imported only when the optional three-dimensional view is enabled.
