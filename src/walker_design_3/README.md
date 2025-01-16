# **What is new?**
- The simulations in this package assume locked knees, since simulations with knees in walker_design_2 did not deliver great results.
- Jinja templating for simulations of various masses and joint angles was omitted, since the templates have to be tediously edited by hand.
- In lieu of Jinja, the "templating" can be done with functions defined in the module *xml_functions.py*.
# **Problems**
- The walker's revolute joints do not behave as desired, as the initial stance leg (in this case the left leg link) is defined as the base link.
- In URDF, only the child link can revolve around the parent link and not vice versa.
- For future designs, the hip link should be defined as the base link having 2 children (left and right leg).
