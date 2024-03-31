from setuptools import setup
from setuptools import find_packages

PKG_NAME    = 'um982-driver'
VERSION     = '0.1.2'
DESCRIPTION = """
- Driver for UNICORECOMM UM982/UM980 GPS
- Support NMEA, extended NMEA Instruction Set: `PVTSLN`, `KSXT`, `GNHPR`, `BESTNAV`
- Support ASSIC Instruction Set Only now
- Get as much location information as possible ( more than stantad NMEA sentence )
"""

setup(
    name        = PKG_NAME,
    version     = VERSION,
    description = DESCRIPTION,
    packages    = find_packages(),
    zip_safe    = False,
    project_urls={
        "Documentation": "https://github.com/sunshineharry/UM982Driver/",
        "Code":          "https://github.com/sunshineharry/UM982Driver/",
    },
    install_requires=[
        "numpy",
        "pyproj"
    ],
)
