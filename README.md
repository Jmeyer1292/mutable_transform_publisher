# Mutable Transform Publisher

A ROS Node in the spirit of tf's static_transform_publisher but with a service to set the transform while running and the ability to persist changes if desired.

## Dependencies

 * ROS
 * YAML-CPP

## Usage
  
  1. Create a yaml file with the following syntax:
  ```yaml
  - parent: foo
    child: bar
    x: 0
    y: 1
    z: 2
    qx: 0
    qz: 0
    qy: 0
    qw: 1
  - parent: foo
    child: baz
    x: 0
    y: 0
    z: 0
    qx: 0
    qy: 0
    qz: 0
    qw: 1
  ```
  2. Run the `mutable_transform_pub` node in `mutable_transform_publisher` and set the `yaml_path` argument.
  ```bash
rosrun mutable_transform_publisher mutable_transform_pub _yaml_path:=<PATH_TO_YOUR_YAML>
  ```

  3. While running, you can call the `set_transform` service of type `mutable_transform_publisher::SetTranform`.
  ```bash
  rosservice call /set_transform "transform:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'foo'
  child_frame_id: 'bar'
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0"
  ```

  4. You can also use the `set_transform` service to add a new TF transform.

  5. When ROS shutsdown, if you have set the `commit` parameter (which defaults to true) the state of your publisher will be saved back to the yaml path.
  
## Issues
  1. Need interface to set period of publishing. Should it be parameter, in the service call, or both?
  2. Need to make an explicit 'CreateTransform' service.
