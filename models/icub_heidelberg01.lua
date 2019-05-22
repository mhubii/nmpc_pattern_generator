scaling_factor = 0.001

meshes = {
	root_link = {
		name = "root_link",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = 90.},
		translate = {-0.025, 0.0, 0.02},
    color = { 0.9, 0.2, 0.2},
    src = "obj_meshes_meshup/sim_root_link_prt.obj",
	},
  lap_belt_1 = {
		name = "lap_belt_1",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {0.009, -0.005, -0.05},
		mesh_center = {0.0393999998093, 0.032, 0.0364},
    src = "obj_meshes_meshup/sim_lap_belt_1_prt.obj",
	},
	lap_belt_2 = {
		name = "lap_belt_2",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {-0.028, -0.005, -0.05},
		mesh_center = {0, 0, 0.0902389999948},
    src = "obj_meshes_meshup/sim_lap_belt_2_prt.obj",
	},
	r_hip_1 = {
		name = "r_hip_1",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {0.135, -0.055, -0.05},
    mesh_center = {0.0262461, 0.151913, 0.0428515000001},
    src = "obj_meshes_meshup/sim_sea_r_hip_1_prt.obj",
	},
	r_hip_2 = {
		name = "r_hip_2",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {0.178, -0.076, -0.05},
		mesh_center = {0.0701, 0.151913, 0.0690985998094},
    src = "obj_meshes_meshup/sim_sea_r_hip_2_prt.obj",
	},
	r_upper_thigh = {
		name = "r_upper_thigh",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
    src = "obj_meshes_meshup/sim_sea_r_upper_thigh_prt.obj",
	},
	r_thigh = {
		name = "r_thigh",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {0.3, -0.065, -0.05},
		mesh_center = {0.0701, 0.241013, 0.0437089998094},
    color = { 0.9, 0.2, 0.2},
    src = "obj_meshes_meshup/sim_sea_r_thigh_prt.obj",
	},
	r_shank = {
		name = "r_shank",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {0.485, -0.065, -0.05},
		mesh_center = {0.0841895, 0.386638, 0.0437089998094},
    color = { 0.9, 0.2, 0.2},
    src = "obj_meshes_meshup/sim_sea_r_shank_prt.obj",
	},
	r_ankle = {
		name = "r_ankle",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {0.595, -0.065, -0.05},
		mesh_center = {0.09712, 0.587138, 0.0437089998094},
    src = "obj_meshes_meshup/sim_sea_r_ankle_prt.obj",
	},
	r_upper_foot = {
		name = "r_upper_foot",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {0.605, -0.065, -0.05},
    mesh_center = {0.0701, 0.587138, -0.000853467381356},
    src = "obj_meshes_meshup/sim_sea_r_foot_prt.obj",
	},
	r_foot = {
		name = "r_foot",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 1., 0.}, angle = 90.},
		translate = {0.04, 0.07, 0.662},
    color = { 0.9, 0.2, 0.2},
    src = "obj_meshes_meshup/sim_sea_r_sole_prt.obj",
	},
	l_hip_1 = {
		name = "l_hip_1",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {0.188, -0.055, 0.05},
		mesh_center = {-0.026, 0.151913, -0.0428515},
    src = "obj_meshes_meshup/sim_sea_l_hip_1_prt.obj",
	},
	l_hip_2 = {
		name = "l_hip_2",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {0.178, -0.076, 0.05},
		mesh_center = {0.0701, 0.151913, -0.0690985998093},
    src = "obj_meshes_meshup/sim_sea_l_hip_2_prt.obj",
	},
	l_upper_thigh = {
		name = "l_upper_thigh",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
    src = "obj_meshes_meshup/sim_sea_l_upper_thigh_prt.obj",
	},
	l_thigh = {
		name = "l_thigh",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {0.3, -0.065, 0.05},
		mesh_center = {0.0701, 0.241013, -0.0437089998092},
    color = { 0.9, 0.2, 0.2},
    src = "obj_meshes_meshup/sim_sea_l_thigh_prt.obj",
	},
	l_shank = {
		name = "l_shank",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		mesh_center = {0.0841895, 0.386638, -0.043708999809},
		translate = {0.485, -0.065, 0.05},
    color = { 0.9, 0.2, 0.2},
    src = "obj_meshes_meshup/sim_sea_l_shank_prt.obj",
	},
	l_ankle = {
		name = "l_ankle",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {0.595, -0.065, 0.05},
		mesh_center = {0.09712, 0.587138, -0.0437089998087},
    src = "obj_meshes_meshup/sim_sea_l_ankle_prt.obj",
	},
	l_upper_foot = {
		name = "l_upper_foot",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {0.605, -0.065, 0.05},
		mesh_center = {0.0701, 0.587138, 0.000853467382004},
    src = "obj_meshes_meshup/sim_sea_l_foot_prt.obj",
	},
	l_foot = {
		name = "l_foot",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 1., 0.}, angle = -90.},
		translate = {0.04, -0.07, 0.662},
    color = { 0.9, 0.2, 0.2},
    src = "obj_meshes_meshup/sim_sea_l_sole_prt.obj",
	},
	chest = {
		name = "chest",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    rotate = { axis = {0., 0., 1.}, angle = -90.},
		translate = {-0.175, -0.155, -0.03},
		mesh_center = {-0.000635442809265, 0.094882000002, -0.0123862999847},
    color = { 0.9, 0.2, 0.2},
    src = "obj_meshes_meshup/sim_chest__prt.obj",
	},
	weight1kg = {
		name = "weight1kg",
		scale = {scaling_factor, scaling_factor, scaling_factor},
    color = { 0.9, 0.2, 0.2},
    src = "obj_meshes_meshup/weight-1kg_prt.dae.obj",
	}, 
}

model = {
  gravity = { 0, 0, -9.81,},
  configuration = {
    axis_right = { 0, -1, 0,},
    axis_front = { 1, 0, 0,},
    axis_up = { 0, 0, 1,},
  },
  frames = {
    {
      parent = "ROOT",
      joint = {
        { 0, 0, 0, 1, 0, 0,},
        { 0, 0, 0, 0, 1, 0,},
        { 0, 0, 0, 0, 0, 1,},
        { 1, 0, 0, 0, 0, 0,},
        { 0, 1, 0, 0, 0, 0,},
        { 0, 0, 1, 0, 0, 0,},
      },
      name = "root_link",
      body = {
        inertia = {
          { 0.0252712, -8.71849e-05, 0.00172034,},
          { -8.71849e-05, 0.0140911, -6.63933e-05,},
          { 0.00172034, -6.63933e-05, 0.0205529,},
        },
        mass = 5.1504,
        com = { 0.0251137, 0.000104218, -0.0437451,},
      },
			visuals = {
				meshes.root_link,
			},
    },
    {
      joint_frame = {
        E = {
          { 4.8965888601467e-12, 1, 5.5511151231258e-17,},
          { 0.0067194714286016, -3.2957998841382e-14, 0.99997742409702,},
          { 0.99997742409702, -4.8964779422261e-12, -0.0067194714286016,},
        },
        r = { 0.0064515, -0.0111, -0.119913,},
      },
      name = "l_hip_1",
      joint = {
        { 1, 0, 0, 0, 0, 0,},
      },
      parent = "root_link",
      body = {
        inertia = {
          { 0.000403391, -1.46806806969e-06, -2.86703008435e-06,},
          { -1.46806806969e-06, 0.000572459526705, 2.05246847003e-07,},
          { -2.86703008435e-06, 2.05246847003e-07, 0.000577587473295,},
        },
        mass = 0.9279,
        com = { -0.0529681, -3.63786457601e-05, 0.000539056614961,},
      },
			visuals = {
				meshes.l_hip_1,
			},
    },
    {
      joint_frame = {
        r = { -0.059, -8.59963575406e-07, 0.026362200929,},
      },
      name = "l_hip_2",
      joint = {
        { -0, 0, -1, 0, 0, 0,},
      },
      parent = "l_hip_1",
      body = {
        inertia = {
          { 0.00066689083275023, 8.8566625353674e-07, -1.4793549176652e-07,},
          { 8.8566625353674e-07, 0.0003430672817575, -1.2764390221182e-05,},
          { -1.4793549176652e-07, -1.2764390221182e-05, 0.00049418212361891,},
        },
        mass = 0.5446,
        com = { 6.810624312412e-05, -0.053244500550208, -0.024717650585675,},
      },
		visuals = {
		  meshes.l_hip_2,
		},
    },
    {
      joint_frame = {
        E = {
          { 1, 4.8575887640574e-12, -4.896644470667e-12,},
          { -4.8575887640574e-12, 1, 6.2172489379009e-15,},
          { 4.896644470667e-12, -6.2172489379009e-15, 1,},
        },
        r = { 8.8026967089725e-14, -0.092299461659665, -0.025504672195776,},
      },
      name = "l_upper_leg",
      joint = {
        { -0, 1, 0, 0, 0, 0,},
      },
      parent = "l_hip_2",
      body = {
        inertia = {
          { 0.00794621, 0.000169714741039, -1.07285855603e-05,},
          { 0.000169714741039, 0.00179481538882, 0.000204937091082,},
          { -1.07285855603e-05, 0.000204937091082, 0.00796710461118,},
        },
        mass = 2.2544,
        com = { -5.88999999999e-05, -0.0791582282008, -0.0021491470665,},
      },
      visuals = {
        meshes.l_thigh,
      },
    },
    {
      joint_frame = {
        r = { -0.017957, -0.145625287455, 0,},
      },
      name = "l_lower_leg",
      joint = {
        { 1, 0, 0, 0, 0, 0,},
      },
      parent = "l_upper_leg",
      body = {
        inertia = {
          { 0.00565145, 0.000651870769773, -1.78506300043e-05,},
          { 0.000651870769773, 0.00171182385437, 0.000177423555915,},
          { -1.78506300043e-05, 0.000177423555915, 0.00615193614563,},
        },
        mass = 1.7086,
        com = { 0.0117639, -0.0833608525374, -0.000293265754161,},
      },
		    visuals = {
		      meshes.l_shank,
		    },
    },
    {
      joint_frame = {
        E = {
          { 1, 0, 0,},
          { 0, 0.99999999971008, 2.4080004128673e-05,},
          { 0, -2.4080004128673e-05, 0.99999999971008,},
        },
        r = { -0.021343, -0.200983719847, 0.00294927094483,},
      },
      name = "l_ankle_1",
      joint = {
        { 1, 0, 0, 0, 0, 0,},
      },
      parent = "l_lower_leg",
      body = {
        inertia = {
          { 0.000539928, -3.69491078227e-07, 3.92721693901e-05,},
          { -3.69491078227e-07, 0.000581685450236, -2.38798297277e-06,},
          { 3.92721693901e-05, -2.38798297277e-06, 0.000414658549764,},
        },
        mass = 0.8929,
        com = { 0.034639, -0.000149240603678, -0.0151607383132,},
      },
		    visuals = {
		      meshes.l_ankle,
		    },
    },
    {
      joint_frame = {
        r = { 0.0355, -1.6398477099e-05, -0.0475117797388,},
      },
      name = "l_ankle_2",
      joint = {
        { -0, 0, -1, 0, 0, 0,},
      },
      parent = "l_ankle_1",
      body = {
        inertia = {
          { 0.0017509570645868, -1.7796613301344e-05, 3.3325708902202e-05,},
          { -1.7796613301344e-05, 0.0018036534326522, -9.8174899919498e-05,},
          { 3.3325708902202e-05, -9.8174899919498e-05, 0.00050704970056477,},
        },
        mass = 0.6469,
        com = { -0.0031732487556064, -0.05059020039209, 0.029714931500218,},
      },
		    visuals = {
		      meshes.l_upper_foot,
		    },
    },
    {
      joint_frame = {
        E = {
          { 4.8965888601467e-12, -1, -5.5511151231258e-17,},
          { 4.8965888601467e-12, -5.5511127254675e-17, 1,},
          { -1, -4.8965888601467e-12, 4.8965888601467e-12,},
        },
        r = { 0.0064515, 0.0111, -0.119913,},
      },
      name = "r_hip_1",
      joint = {
        { 1, 0, 0, 0, 0, 0,},
      },
      parent = "root_link",
      body = {
        inertia = {
          { 0.000403391, -1.44877e-06, 2.87683e-06,},
          { -1.44877e-06, 0.000572457, -1.70772e-07,},
          { 2.87683e-06, -1.70772e-07, 0.00057759,},
        },
        mass = 0.9279,
        com = { -0.0529681, -4e-05, -0.0005388,},
      },
		    visuals = {
		      meshes.r_hip_1,
		    },
    },
    {
      joint_frame = {
        r = { -0.059, 0, -0.0263622,},
      },
      name = "r_hip_2",
      joint = {
        { 0, 0, 1, 0, 0, 0,},
      },
      parent = "r_hip_1",
      body = {
        inertia = {
          { 0.00066689083275096, 8.8664030708957e-07, 1.4198094290537e-07,},
          { 8.8664030708957e-07, 0.00034324564082643, 1.3778626501743e-05,},
          { 1.4198094290537e-07, 1.3778626501743e-05, 0.00049400376455071,},
        },
        mass = 0.5446,
        com = { 6.8106243114205e-05, -0.053255208960705, 0.025075467462358,},
      },
		    visuals = {
		      meshes.r_hip_2,
		    },
    },
    {
      joint_frame = {
        E = {
          { 1, -7.1850416783793e-17, 4.8965276278068e-12,},
          { 7.1850392807511e-17, 1, 4.8965888608409e-12,},
          { -4.8965276278068e-12, -4.8965888608409e-12, 1,},
        },
        r = { -2.9410677994564e-19, -0.0923, 0.0255047,},
      },
      name = "r_upper_leg",
      joint = {
        { 0, -1, 0, 0, 0, 0,},
      },
      parent = "r_hip_2",
      body = {
        inertia = {
          { 0.00794621, 0.000169783, 9.58795e-06,},
          { 0.000169783, 0.00179234, -0.000163445,},
          { 9.58795e-06, -0.000163445, 0.00796958,},
        },
        mass = 2.2544,
        com = { -5.89e-05, -0.079146, 0.0033012,},
      },
		    visuals = {
		      meshes.r_thigh,
		    },
    },
    {
      joint_frame = {
        r = { -0.017957, -0.145625, 0,},
      },
      name = "r_lower_leg",
      joint = {
        { -1, 0, 0, 0, 0, 0,},
      },
      parent = "r_upper_leg",
      body = {
        inertia = {
          { 0.00565145, 0.000651976, 1.347e-05,},
          { 0.000651976, 0.00170964, -0.000147573,},
          { 1.347e-05, -0.000147573, 0.00615412,},
        },
        mass = 1.7086,
        com = { 0.0117639, -0.083358, 0.0024521,},
      },
		    visuals = {
		      meshes.r_shank,
		    },
    },
    {
      joint_frame = {
        E = {
          { 1, 0, 0,},
          { 0, 0.9999999997101, -2.4078899996473e-05,},
          { 0, 2.4078899996473e-05, 0.9999999997101,},
        },
        r = { -0.021343, -0.201, 0,},
      },
      name = "r_ankle_1",
      joint = {
        { 1, 0, 0, 0, 0, 0,},
      },
      parent = "r_lower_leg",
      body = {
        inertia = {
          { 0.000539928, -6.33371e-07, -3.92688e-05,},
          { -6.33371e-07, 0.00058171, 1.26546e-06,},
          { -3.92688e-05, 1.26546e-06, 0.000414634,},
        },
        mass = 0.8929,
        com = { 0.034639, -4.73650698208e-05, 0.0151613988639,},
      },
		    visuals = {
		      meshes.r_ankle,
		    },
    },
    {
      joint_frame = {
        r = { 0.0355, 0, 0.0445624670112,},
      },
      name = "r_ankle_2",
      joint = {
        { 0, 0, -1, 0, 0, 0,},
      },
      parent = "r_ankle_1",
      body = {
        inertia = {
          { 0.0017509570679969, -1.8020142277578e-05, -3.3205372688516e-05,},
          { -1.8020142277578e-05, 0.0018049142269855, 8.9453756800782e-05,},
          { -3.3205372688516e-05, 8.9453756800782e-05, 0.00050578890964763,},
        },
        mass = 0.6469,
        com = { -0.0031732487556036, -0.050485798221468, -0.02642597089189,},
      },
		    visuals = {
		      meshes.r_upper_foot,
		    },
    },
    {
      joint_frame = {
        E = {
          { 4.8965888601467e-12, -1, -5.5511151231258e-17,},
          { 1.6489660209861e-10, -5.5510343800393e-17, 1,},
          { -1, -4.8965888601468e-12, 1.6489660209861e-10,},
        },
        r = { 0, 0.0394, 0,},
      },
      name = "torso_1",
      joint = {
        { 1, 0, 0, 0, 0, 0,},
      },
      parent = "root_link",
      body = {
        inertia = {
          { 0.000416742, -8.88072e-07, 1.08936e-07,},
          { -8.88072e-07, 0.000392401, 5.37987e-06,},
          { 1.08936e-07, 5.37987e-06, 0.000307697,},
        },
        mass = 0.7112,
        com = { 0.0393679204, 0.0293082, 0.000142300004739,},
      },
		    visuals = {
		      meshes.lap_belt_1,
		    },
    },
    {
      joint_frame = {
        r = { 0.0393999998093, 0.032, -0.0587918999948,},
      },
      name = "torso_2",
      joint = {
        { 0, 0, -1, 0, 0, 0,},
      },
      parent = "torso_1",
      body = {
        inertia = {
          { 0.000620392, 3.75632e-07, 1.15418e-07,},
          { 3.75632e-07, 0.000446223, 2.06978e-05,},
          { 1.15418e-07, 2.06978e-05, 0.00039084,},
        },
        mass = 0.3954,
        com = { -0.000121587809265, 0.0287460000001, 0.0580375000046,},
      },
		    visuals = {
		      meshes.lap_belt_2,
		    },
    },
    {
      joint_frame = {
        r = { 0, 0.0514999999991, 0.0642919000083,},
      },
      name = "chest",
      joint = {
        { 0, -1, 0, 0, 0, 0,},
      },
      parent = "torso_2",
      body = {
        inertia = {
          { 0.0252769, 1.73516e-05, -0.0001136,},
          { 1.73516e-05, 0.0497471, 0.000340001,},
          { -0.0001136, 0.000340001, 0.0580977,},
        },
        mass = 6.1306,
        com = { -0.000279596809265, 0.0998630000009, -0.00564929998385,},
      },
		    visuals = {
		      meshes.chest,
		    },
    },
    {
      parent = "l_hip_2",
      joint_frame = {
        E = {
          { 3.2917409166109e-14, -0.0067194714283888, -0.99997742409703,},
          { 1, 4.8905482549201e-12, 5.5511151231206e-17,},
          { 4.8904374733716e-12, -0.99997742409703, 0.0067194714283888,},
        },
        r = { 0, -0.0743000549473, -0.0255046186937,},
      },
      name = "l_hip_3",
    },
    {
      parent = "l_ankle_2",
      joint_frame = {
        E = {
          { 3.2973977688895e-14, -0.0067435508828372, -0.99997726200224,},
          { 1, 4.8897054773941e-12, 0,},
          { 4.8895942952819e-12, -0.99997726200224, 0.0067435508828372,},
        },
        r = { 0, -0.0603002589183, 0.0480625407102,},
      },
      name = "l_foot",
    },
    {
      parent = "l_ankle_2",
      joint_frame = {
        E = {
          { -1.2768261175852e-18, 2.6112536296793e-07, -0.99999999999997,},
          { -1, -5.0963294246543e-12, -5.3954753226312e-20,},
          { -5.0963294246541e-12, 0.99999999999997, 2.6112536296793e-07,},
        },
        r = { 7.0408473328395e-14, -0.074699586987016, 0.048062443309495,},
      },
      name = "l_sole",
      visuals = {
        meshes.l_foot,
      },
    },
    {
      parent = "r_hip_2",
      joint_frame = {
        E = {
          { 6.1232339953433e-17, -6.9417338314041e-22, 1,},
          { -1, 1.6339265552535e-17, 6.1232339953433e-17,},
          { -1.6339265552535e-17, -1, -6.9417338313941e-22,},
        },
        r = { 0, -0.0743, 0.0255047,},
      },
      name = "r_hip_3",
    },
    {
      parent = "r_ankle_2",
      joint_frame = {
        E = {
          { 2.9034414507298e-15, -1.490116125508e-08, 1,},
          { -1, 6.1232302924589e-17, 2.9034414516423e-15,},
          { -6.1232355864148e-17, -1, -1.490116125508e-08,},
        },
        r = { 0, -0.0603008427249, -0.0480625189919,},
      },
      name = "r_foot",
    },
    {
      parent = "r_ankle_2",
      joint_frame = {
        E = {
          { 2.9034414507298e-15, -1.490116125508e-08, 1,},
          { 1, -2.0668517956312e-13, -2.903444530579e-15,},
          { 2.0668517961606e-13, 1, 1.490116125508e-08,},
        },
        r = { -8.8174592478051e-19, -0.0747008427304, -0.048062519206477,},
      },
      name = "r_sole",
      visuals = {
        meshes.r_foot,
      },
    },
  },
}

return model
