# recorder

ROS 2 node for recording video streams.

## Usage

This package builds the executable `recorder` than can be started very easily with `ros2 run` or with launch files, as you prefer.

By specifying multiple topic names via the related node parameter, the node will record all the streams in parallel. Each frame will be saved in a separate file, using the path provided with the appropriate node parameter.

The file name will be composed as the concatenation of `frame_id` and timestamp of the related message, substituting the `/` character with `_`.

### Parameters

See [`params.yaml`](src/params.yaml).

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata
