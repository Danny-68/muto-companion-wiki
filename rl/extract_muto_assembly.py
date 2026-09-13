#!/usr/bin/env python3
"""extract_muto_assembly.py -- draait op de WSL2/RTX5080-machine.
Leest MutoRS.STEP via OpenCASCADE's XCAF-documentmodel (bewaart de
assembly-hierarchie + onderdeelnamen, i.t.t. cadquery's platte import) zodat
de 2341 shells/11075 faces gegroepeerd kunnen worden tot de echte starre
lichamen (chassis + 3 segmenten x 6 poten), als voorbereiding op Fase 7's
Muto-MJCF.
"""
import sys

from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.TopAbs import TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE
from OCP.TopExp import TopExp_Explorer

STEP_PATH = "/home/meinds/projects/muto_rs/yahboom_hardware/5.About hardware/3D_Model_File/MutoRS.STEP"


def count_subshapes(shape):
    counts = {}
    for name, kind in [("SOLID", TopAbs_SOLID), ("SHELL", TopAbs_SHELL), ("FACE", TopAbs_FACE)]:
        exp = TopExp_Explorer(shape, kind)
        n = 0
        while exp.More():
            n += 1
            exp.Next()
        counts[name] = n
    return counts


def label_name(label):
    name_attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), name_attr):
        return name_attr.Get().ToExtString()
    return "(naamloos)"


def main():
    doc = TDocStd_Document(TCollection_ExtendedString("muto-doc"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.SetLayerMode(True)

    status = reader.ReadFile(STEP_PATH)
    print(f"ReadFile status: {status}")
    if status != 1:  # IFSelect_RetDone == 1
        print("FOUT bij lezen STEP-bestand")
        sys.exit(1)

    ok = reader.Transfer(doc)
    print(f"Transfer naar XCAF-document: {ok}")

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    free_shapes = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free_shapes)
    print(f"\nAantal top-level vrije shapes: {free_shapes.Length()}")

    def walk(label, depth=0):
        name = label_name(label)
        shape = shape_tool.GetShape_s(label)
        counts = count_subshapes(shape)
        is_assembly = shape_tool.IsAssembly_s(label)
        indent = "  " * depth
        print(f"{indent}- {name} [assembly={is_assembly}] solids={counts['SOLID']} shells={counts['SHELL']} faces={counts['FACE']}")

        if is_assembly:
            children = TDF_LabelSequence()
            shape_tool.GetComponents_s(label, children)
            for i in range(1, children.Length() + 1):
                child_label = children.Value(i)
                ref_label = child_label  # component label; referred shape via GetReferredShape
                referred = ref_label
                if shape_tool.IsReference_s(child_label):
                    from OCP.TDF import TDF_Label
                    target = TDF_Label()
                    shape_tool.GetReferredShape_s(child_label, target)
                    referred = target
                walk(referred, depth + 1)

    for i in range(1, free_shapes.Length() + 1):
        walk(free_shapes.Value(i))


if __name__ == "__main__":
    main()
