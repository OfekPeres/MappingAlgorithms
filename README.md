# MappingAlgorithms
This repo contains a variety of mapping algorithms in both continuous and discrete space as well as utility classes like a KDTree implementation to vastly improve performance nearest neighbor searches from linear to logarithmic.

See the live docs at: [https://ofekperes.github.io/MappingAlgorithms/](https://ofekperes.github.io/MappingAlgorithms/)


# First time generating initial Sphinx docs
Run the following line from the root project directory
```
sphinx-apidoc -F -A "Mapping Algorithms" -V "0.1" -o docs src
```

In the docs/ directory, delete the src.rst file as it is not needed

## Building the html docs
From the docs/ directory, run:
```
make html
```

# Important Information 
Use Python 3.9 and to install required packages use: 
```
pip install -r requirements.txt
```