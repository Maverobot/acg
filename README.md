# acg
A simple 2D simulator based on Box2D

## Troubleshooting

> Error compiling shader of type 35633!
> 0:1(10): error: GLSL 3.30 is not supported. Supported versions are: 1.10, 1.20, 1.30, 1.00 ES, 3.00 ES, 3.10 ES, and 3.20 ES

Execute the following line in the terminal before running the examples

```sh
export MESA_GL_VERSION_OVERRIDE=3.3
```
