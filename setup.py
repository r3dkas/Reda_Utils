import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="Reda_Utils",
    version="1.0.2",
    author="Reda Kastite",
    author_email="reda_kastite@hotmail.com",
    description="Paquete funciones utiles Navegacion Robot",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/r3dkas/Reda_Utils",
    packages=setuptools.find_packages(),
    classifiers=(
        "Programming Language :: Python :: 2",
        "License :: OSI Approved :: BSD License",
        "Operating System :: Unix",
    ),
)