#!/usr/bin/env python3
"""
generate_visual_markers.py — Visual Topological Graph & Rack Bay Divider Generator
====================================================================================
Generates collision-free SDF visual markers for:
  1. Equidistant Floor Graph Nodes (Green/Cyan Floor Markers at 0.5m from rack bounds).
     Top staging area nodes at X = -0.75m for amr_1 (Y=7.5), amr_2 (Y=5.0), amr_3 (Y=2.5).
  2. Rack Bay Dividers on 4 sides of each 2m x 4m rack:
     - Long Faces (4.0m length): 2 vertical stripe dividers creating 3 equal 1.33m bays.
       * Top Long Face (X = cx - 1.0): Bay_1, Bay_2, Bay_3
       * Bottom Long Face (X = cx + 1.0): Bay_6, Bay_7, Bay_8
     - Short Faces (2.0m length): 1 vertical stripe divider creating 2 equal 1.0m bays.
       * Right Short Face (Y = cy - 2.0): Bay_4, Bay_5
       * Left Short Face (Y = cy + 2.0): Bay_9, Bay_10

All generated markers strictly lack <collision> tags to prevent interference with AMR physics/LiDAR.
"""

def generate_visual_sdf() -> str:
    sdf_snippets = []

    # ------------------------------------------------------------------
    # 1. Floor Graph Visual Nodes (Green Floor Markers)
    # ------------------------------------------------------------------
    x_aisles = [-0.75, 0.0, 3.0, 6.0, 9.0, 11.0, 12.0, 12.5, 13.0, 14.0]
    y_corridors = [2.5, 5.0, 7.5, 12.5, 13.5]

    nodes = {}

    # Intersection nodes
    for x in x_aisles:
        for y in y_corridors:
            nodes[f"Int_X{x}_Y{y}"] = (x, y)

    # Racks & 10 Bays per Rack
    racks = [
        ("Rack_Yellow_1", 1.5, 10.0),
        ("Rack_Yellow_2", 4.5, 10.0),
        ("Rack_Yellow_3", 7.5, 10.0),
        ("Rack_Yellow_4", 10.5, 10.0),
        ("Rack_Blue_1", 1.5, 5.0),
        ("Rack_Blue_2", 4.5, 5.0),
        ("Rack_Blue_3", 7.5, 5.0),
        ("Rack_Blue_4", 10.5, 5.0),
    ]

    for rack_name, cx, cy in racks:
        # Long Face 1 (Top Long Edge, X = cx - 1.0, 3 Bays: 1, 2, 3 in aisle X = cx - 1.5)
        nodes[f"{rack_name}_Bay_1"] = (round(cx - 1.5, 2), round(cy + 1.2, 2))
        nodes[f"{rack_name}_Bay_2"] = (round(cx - 1.5, 2), round(cy, 2))
        nodes[f"{rack_name}_Bay_3"] = (round(cx - 1.5, 2), round(cy - 1.2, 2))

        # Short Face 1 (Right Short Edge, Y = cy - 2.0, 2 Bays: 4, 5 in corridor Y = cy - 2.5)
        nodes[f"{rack_name}_Bay_4"] = (round(cx - 0.5, 2), round(cy - 2.5, 2))
        nodes[f"{rack_name}_Bay_5"] = (round(cx + 0.5, 2), round(cy - 2.5, 2))

        # Long Face 2 (Bottom Long Edge, X = cx + 1.0, 3 Bays: 6, 7, 8 in aisle X = cx + 1.5)
        nodes[f"{rack_name}_Bay_6"] = (round(cx + 1.5, 2), round(cy - 1.2, 2))
        nodes[f"{rack_name}_Bay_7"] = (round(cx + 1.5, 2), round(cy, 2))
        nodes[f"{rack_name}_Bay_8"] = (round(cx + 1.5, 2), round(cy + 1.2, 2))

        # Short Face 2 (Left Short Edge, Y = cy + 2.0, 2 Bays: 9, 10 in corridor Y = cy + 2.5)
        nodes[f"{rack_name}_Bay_9"] = (round(cx + 0.5, 2), round(cy + 2.5, 2))
        nodes[f"{rack_name}_Bay_10"] = (round(cx - 0.5, 2), round(cy + 2.5, 2))

    sdf_snippets.append("    <!-- ===================== Visual Topological Graph Floor Markers (Green Dots - NO COLLISION) ===================== -->")
    
    for idx, (name, (x, y)) in enumerate(nodes.items()):
        marker = f"""    <model name="vis_node_{idx}">
      <static>true</static>
      <pose>{x} {y} 0.005 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><cylinder><radius>0.12</radius><length>0.005</length></cylinder></geometry>
          <material>
            <ambient>0.0 0.8 0.3 1</ambient>
            <diffuse>0.0 1.0 0.4 1</diffuse>
            <emissive>0.0 0.4 0.15 1</emissive>
          </material>
        </visual>
      </link>
    </model>"""
        sdf_snippets.append(marker)

    # ------------------------------------------------------------------
    # 2. Rack Bay Subdivision Markers (Visual Dividers on 4 Rack Edges - NO COLLISION)
    # ------------------------------------------------------------------
    sdf_snippets.append("\n    <!-- ===================== Visual Rack Bay Subdivision Markers (NO COLLISION) ===================== -->")

    divider_idx = 0
    for rack_name, cx, cy in racks:
        is_yellow = "Yellow" in rack_name
        color_ambient = "0.15 0.15 0.15 1" if is_yellow else "0.85 0.85 0.85 1"
        color_diffuse = "0.25 0.25 0.25 1" if is_yellow else "0.95 0.95 0.95 1"

        # Dividers on Top Long Edge (X = cx - 1.01, length 4.0m in Y): 2 dividers at y = cy - 0.67 and y = cy + 0.67
        for div_y in [cy - 0.67, cy + 0.67]:
            div_model = f"""    <model name="vis_div_{divider_idx}">
      <static>true</static>
      <pose>{cx - 1.01} {div_y:.2f} 0.75 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><box><size>0.04 0.04 1.48</size></box></geometry>
          <material><ambient>{color_ambient}</ambient><diffuse>{color_diffuse}</diffuse></material>
        </visual>
      </link>
    </model>"""
            sdf_snippets.append(div_model)
            divider_idx += 1

        # Divider on Right Short Edge (Y = cy - 2.01, length 2.0m in X): 1 divider at x = cx
        div_model = f"""    <model name="vis_div_{divider_idx}">
      <static>true</static>
      <pose>{cx} {cy - 2.01} 0.75 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><box><size>0.04 0.04 1.48</size></box></geometry>
          <material><ambient>{color_ambient}</ambient><diffuse>{color_diffuse}</diffuse></material>
        </visual>
      </link>
    </model>"""
        sdf_snippets.append(div_model)
        divider_idx += 1

        # Dividers on Bottom Long Edge (X = cx + 1.01, length 4.0m in Y): 2 dividers at y = cy - 0.67 and y = cy + 0.67
        for div_y in [cy - 0.67, cy + 0.67]:
            div_model = f"""    <model name="vis_div_{divider_idx}">
      <static>true</static>
      <pose>{cx + 1.01} {div_y:.2f} 0.75 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><box><size>0.04 0.04 1.48</size></box></geometry>
          <material><ambient>{color_ambient}</ambient><diffuse>{color_diffuse}</diffuse></material>
        </visual>
      </link>
    </model>"""
            sdf_snippets.append(div_model)
            divider_idx += 1

        # Divider on Left Short Edge (Y = cy + 2.01, length 2.0m in X): 1 divider at x = cx
        div_model = f"""    <model name="vis_div_{divider_idx}">
      <static>true</static>
      <pose>{cx} {cy + 2.01} 0.75 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><box><size>0.04 0.04 1.48</size></box></geometry>
          <material><ambient>{color_ambient}</ambient><diffuse>{color_diffuse}</diffuse></material>
        </visual>
      </link>
    </model>"""
        sdf_snippets.append(div_model)
        divider_idx += 1

    return "\n".join(sdf_snippets)

if __name__ == "__main__":
    print(generate_visual_sdf())
