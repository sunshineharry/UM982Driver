from setuptools import setup
from setuptools import find_packages

PKG_NAME    = 'um982-driver'
VERSION     = '0.2.1'
DESCRIPTION = """
- Driver for UNICORECOMM UM982/UM980 GPS
- Support NMEA, extended NMEA Instruction Set: `PVTSLN`, `GNHPR`, `BESTNAV`
- Support ASSIC Instruction Set Only now
- Get as much location information as possible ( more than stantad NMEA sentence )
"""
with open("../readme.md", "r") as fh:
    LONG_DESCRIPTION = fh.read()

setup(
    name                            = PKG_NAME,
    version                         = VERSION,
    description                     = DESCRIPTION,
    long_description                = LONG_DESCRIPTION,
    long_description_content_type   = "text/markdown",
    packages                        = find_packages(),
    zip_safe                        = False,
    project_urls={
        "Documentation": "https://github.com/sunshineharry/UM982Driver/",
        "Code":          "https://github.com/sunshineharry/UM982Driver/",
    },
    install_requires=[
        "numpy",
        "pyproj"
    ],
)
