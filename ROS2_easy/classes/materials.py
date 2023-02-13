"""all materials a model could use, if it doesnt exist, add it here"""


from dataclasses import dataclass

@dataclass
class Material():
    """
    Represents material properties.
    
    NOTE: FreeCAD does not appear to have a innate way to give models material properties, so materials will need to be defined here.

    See: https://wiki.freecadweb.org/Material
    """
    name: str
    """name of material"""
    density: float
    """
    density of material in grams per cubic centimeter(g/cc)/(g/cm^3)
    
    get density of material from here:
    https://www.matweb.com/
    """

GENERIC_PETG = Material("PETG", 1.319)
"""'Generic' PETG. Should be the average of PETG properties"""