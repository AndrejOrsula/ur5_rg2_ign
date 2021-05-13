#!/usr/bin/env python3

import sys
from os import path, listdir

import trimesh
from pcg_gazebo.parsers.sdf import SDF, create_sdf_element
# Note: Both `trimesh` and `pcg_gazebo` can be installed via `pip`
# `trimesh` is used to estimate volume and inertial properties from meshes of links
# `pcg_gazebo` is used for its SDF parser


def main():

    # Total mass taken from datasheet (given as 18.4kg and 0.78kg respectively for UR5 and RG2)
    ur5_mass = 18.4
    rg2_mass = 0.78
    print(
        f'Estimating inertial properties for each link to add up to {ur5_mass}kg for UR5 and {rg2_mass}kg for RG2')

    # Percentage of mass to redistribute from hand to fingers due to internal mechanical coupling
    # Choose whatever feels right (default value here is guesstimated)
    pct_mass_of_hand = 0.5

    # Get path to all visual meshes
    ur5_visual_mesh_dir = path.join(path.dirname(path.dirname(
        path.realpath(__file__))), 'ur5_rg2', 'meshes', 'visual', 'ur5')
    ur5_visual_mesh_basenames = listdir(ur5_visual_mesh_dir)
    ur5_visual_mesh_basenames.sort()
    rg2_visual_mesh_dir = path.join(path.dirname(path.dirname(
        path.realpath(__file__))), 'ur5_rg2', 'meshes', 'visual', 'rg2')
    rg2_visual_mesh_basenames = listdir(rg2_visual_mesh_dir)
    rg2_visual_mesh_basenames.sort()

    # Load all meshes
    ur5_meshes = {}
    for mesh_basename in ur5_visual_mesh_basenames:
        ur5_link_name = path.splitext(mesh_basename)[0]
        ur5_mesh_path = path.join(ur5_visual_mesh_dir, mesh_basename)
        ur5_meshes[ur5_link_name] = trimesh.load(ur5_mesh_path,
                                                 force='mesh',
                                                 ignore_materials=True)
    rg2_meshes = {}
    for mesh_basename in rg2_visual_mesh_basenames:
        rg2_link_name = path.splitext(mesh_basename)[0]
        rg2_mesh_path = path.join(rg2_visual_mesh_dir, mesh_basename)
        rg2_meshes[rg2_link_name] = trimesh.load(rg2_mesh_path,
                                                 force='mesh',
                                                 ignore_materials=True)

    # Compute the total volume of the robot in order to estimate the required density
    ur5_total_volume = 0.0
    for link_name in ur5_meshes:
        ur5_mesh = ur5_meshes[link_name]
        print('Volume estimate of %s: %f m^3' % (link_name, ur5_mesh.volume))
        ur5_total_volume += ur5_mesh.volume
    rg2_total_volume = 0.0
    for link_name in rg2_meshes:
        rg2_mesh = rg2_meshes[link_name]
        print('Volume estimate of %s: %f m^3' % (link_name, rg2_mesh.volume))
        if link_name == 'finger':
            rg2_total_volume += 2*rg2_mesh.volume
            print('Note: Finger volume added twice to the total volume')
        else:
            rg2_total_volume += rg2_mesh.volume

    # Compute average density
    ur5_average_density = ur5_mass/ur5_total_volume
    print('Average density estimate for UR5: %f kg/m^3' % ur5_average_density)
    rg2_average_density = rg2_mass/rg2_total_volume
    print('Average density estimate for RG2: %f kg/m^3' % rg2_average_density)

    # Estimate inertial properties for each link
    ur5_mass = {}
    ur5_inertia = {}
    ur5_centre_of_mass = {}
    for link_name in ur5_meshes:
        ur5_mesh = ur5_meshes[link_name]
        ur5_mesh.density = ur5_average_density
        ur5_mass[link_name] = ur5_mesh.mass
        ur5_inertia[link_name] = ur5_mesh.moment_inertia
        ur5_centre_of_mass[link_name] = ur5_mesh.center_mass
    rg2_mass = {}
    rg2_inertia = {}
    rg2_centre_of_mass = {}
    for link_name in rg2_meshes:
        rg2_mesh = rg2_meshes[link_name]
        rg2_mesh.density = rg2_average_density
        rg2_mass[link_name] = rg2_mesh.mass
        rg2_inertia[link_name] = rg2_mesh.moment_inertia
        rg2_centre_of_mass[link_name] = rg2_mesh.center_mass

    # Redistribute X% of the hand's mass to the fingers due to internal mechanical coupling
    # This improves reliability of grasps and makes fingers less susceptible to disturbances
    print("Redistributing %f%% of hand's mass fingers" % (100*pct_mass_of_hand))
    old_mass_finger = rg2_mass['finger']
    # Add half of hand's redistributed mass to each finger
    finger_extra_mass = (pct_mass_of_hand/2)*rg2_mass['hand']
    rg2_mass['finger'] += finger_extra_mass
    # Then, update inertial proportionally to mass increase ratio
    rg2_inertia['finger'] *= rg2_mass['finger']/old_mass_finger
    # Recompute centre of finger's mass to account for this redistribution
    # TODO: Read translation directly from SDF (currently copied and hard-coded)
    translation_hand_finger = [0.105, 0.017, 0.0]
    for i in range(3):
        rg2_centre_of_mass['finger'][i] = (old_mass_finger * rg2_centre_of_mass['finger'][i] +
                                       finger_extra_mass * (rg2_centre_of_mass['hand'][i]-translation_hand_finger[i])) / rg2_mass['finger']
    # Reduce mass and inertia of hand to (1.0-X)% in order to account for redistribution of its mass
    rg2_mass['hand'] *= (1.0 - pct_mass_of_hand)
    rg2_inertia['hand'] *= (1.0 - pct_mass_of_hand)

    # Create a new SDF with one model
    sdf = SDF()
    sdf.add_model(name='UR5')
    ur5_model = sdf.models[0]
    sdf.add_model(name='RG2')
    rg2_model = sdf.models[1]

    # Set inertial properties for each link into the SDF
    for link_name in ur5_meshes:
        link = create_sdf_element('link')
        link.mass = ur5_mass[link_name]
        link.inertia.ixx = ur5_inertia[link_name][0][0]
        link.inertia.iyy = ur5_inertia[link_name][1][1]
        link.inertia.izz = ur5_inertia[link_name][2][2]
        link.inertia.ixy = ur5_inertia[link_name][0][1]
        link.inertia.ixz = ur5_inertia[link_name][0][2]
        link.inertia.iyz = ur5_inertia[link_name][1][2]
        link.inertial.pose = [ur5_centre_of_mass[link_name][0],
                              ur5_centre_of_mass[link_name][1],
                              ur5_centre_of_mass[link_name][2],
                              0.0, 0.0, 0.0]
        ur5_model.add_link(link_name, link)
    for link_name in rg2_meshes:
        link = create_sdf_element('link')
        link.mass = rg2_mass[link_name]
        link.inertia.ixx = rg2_inertia[link_name][0][0]
        link.inertia.iyy = rg2_inertia[link_name][1][1]
        link.inertia.izz = rg2_inertia[link_name][2][2]
        link.inertia.ixy = rg2_inertia[link_name][0][1]
        link.inertia.ixz = rg2_inertia[link_name][0][2]
        link.inertia.iyz = rg2_inertia[link_name][1][2]
        link.inertial.pose = [rg2_centre_of_mass[link_name][0],
                              rg2_centre_of_mass[link_name][1],
                              rg2_centre_of_mass[link_name][2],
                              0.0, 0.0, 0.0]
        rg2_model.add_link(link_name, link)

    # Write into output file
    output_file = 'ur5_rg2_inertial_out.sdf'
    sdf.export_xml(output_file)
    print('Results written into "%s"' % output_file)


if __name__ == "__main__":
    main()
