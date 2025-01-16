import xml.etree.ElementTree as ET

def modify_mass_urdf(tree: ET.ElementTree, mass_dict: dict) -> ET.ElementTree:
    """
    Modify the masses and inertial properties of links in a URDF.

    Parameters:
        tree (ET.ElementTree): The parsed XML tree of the URDF.
        mass_dict (dict): A dictionary where keys are link names and values are the new masses.

    Returns:
        ET.ElementTree: The modified XML tree that can be used for further modifications
    """
    root = tree.getroot()

    # Namespace handling if required (URDF typically doesn't use a namespace)
    for link_name, new_mass in mass_dict.items():
        # Find the link element by name
        link = root.find(f".//link[@name='{link_name}']")
        if link is None:
            print(f"Link '{link_name}' not found in the URDF.")
            continue

        # Find the inertial element
        inertial = link.find("inertial")
        if inertial is None:
            print(f"No inertial element found for link '{link_name}'.")
            continue

        # Update mass
        mass_elem = inertial.find("mass")
        if mass_elem is None:
            print(f"No <mass> element found for link '{link_name}'.")
            continue

        old_mass = float(mass_elem.get("value", "0.0"))
        if old_mass == 0:
            print(f"Old mass for link '{link_name}' is zero or not set.")
            continue

        # Calculate mass ratio
        mass_ratio = new_mass / old_mass

        # Update mass element value
        mass_elem.set("value", str(new_mass))

        # Update inertia elements
        inertia = inertial.find("inertia")
        if inertia is None:
            print(f"No <inertia> element found for link '{link_name}'.")
            continue

        for attr in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
            if attr in inertia.attrib:
                old_inertia_value = float(inertia.get(attr, "0.0"))
                new_inertia_value = old_inertia_value * mass_ratio
                inertia.set(attr, str(new_inertia_value))

    return tree


