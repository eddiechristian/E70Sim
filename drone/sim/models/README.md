# Models

Model files for the `view` program (`cargo run --release --bin view`).

## `x500.stl`

The x500, a 500 mm quad on a carbon frame, as the PX4 simulator draws it. It is not the Mark4 V2 10 inch frame of [`quad10.md`](../../../quad10.md): no model of that frame was found that may be put in a public repository. The x500 is close in size, 0.49 m from motor to motor against 0.43 m, and has the same X layout.

- **From:** `models/x500_base/` of [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models) on GitHub.
- **Licence:** BSD 3-Clause, copyright Rudis Laboratories. [`LICENSE-x500`](LICENSE-x500) is the licence file of that directory, and has to stay with the model.
- **Made by:** [`make_x500.py`](make_x500.py), which places the frame, the four motors and the four propellers as the simulator's `model.sdf` says and writes them into one file. The flat pictures of labels on the simulator's model are left out.
- **In it:** 251,556 triangles, in metres, with x forward, y left and z up. Each triangle has its colour in the two spare bytes of the binary `.stl` format: bit 15 set, then five bits each of red, green and blue.
- **Size:** 0.41 m long and 0.30 m high with its legs. The propellers all lie across it, tip to tip 0.64 m.

To make it again, download `model.sdf` and the five files of `meshes/` from that directory into one place, and run:

```
./make_x500.py that-place x500.stl
```

## Another model

`--model` takes an `.obj` file or a binary `.stl` file. The `view` program's `--model-scale`, `--model-up` and `--model-turn` say what its units and axes are. CAD of the Mark4 frame is sold as STEP files, which FreeCAD can save as `.stl`.
