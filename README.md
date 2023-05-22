# Door opening pipeline from Human demonstration

This repository contains a Docker container which involves:
- Robot Vision Library (RVL),
- ROS with packages for controlling the UR5 robot and
- TensorMask from detectron2.

## Installation

### Docker installation

You will need a PC that supports **Nvidia drivers and CUDA** (don't need to install them on your own) in order to launch TensorMask.

#### Ubuntu

Install Docker from the [official page](https://docs.docker.com/engine/install/ubuntu/). Don't forget to do the [postinstall steps](https://docs.docker.com/engine/install/linux-postinstall/).

#### Windows

Install Docker from the [official page](https://docs.docker.com/desktop/install/windows-install/). To run the necessary shell scripts, you can use [WSL](https://learn.microsoft.com/en-us/windows/wsl/install).


### Container setup





Use the package manager [pip](https://pip.pypa.io/en/stable/) to install foobar.

```bash
pip install foobar
```

## Usage

```python
import foobar

# returns 'words'
foobar.pluralize('word')

# returns 'geese'
foobar.pluralize('goose')

# returns 'phenomenon'
foobar.singularize('phenomena')
```

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[MIT](https://choosealicense.com/licenses/mit/)