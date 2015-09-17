/* +---------------------------------------------------------------------------+
   |              OpenBeam - C++ Finite Element Analysis library               |
   |                                                                           |
   |   Copyright (C) 2010-2013  Jose Luis Blanco Claraco                       |
   |                              University of Malaga                         |
   |                                                                           |
   | OpenBeam is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   | OpenBeam is distributed in the hope that it will be useful,               |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with OpenBeam.  If not, see <http://www.gnu.org/licenses/>.     |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <openbeam/openbeam.h>

#include <iostream>

using namespace openbeam;
using namespace std;

int main_code()
{
    openbeam::setVerbosityLevel(1);


	CStructureProblem  problem;



	// Nodes:
    problem.setNumberOfNodes(5);

    problem.setNodePose(0, TRotationTrans3D(0,   0, 0, 0,0,0)  );
    problem.setNodePose(1, TRotationTrans3D(5,   5, 0, 0,0,0)  );
    problem.setNodePose(2, TRotationTrans3D(10,  0, 0, 0,0,0)  );
    problem.setNodePose(3, TRotationTrans3D(15, -5, 0, 0,0,0)  );

	problem.setNodePose(4, TRotationTrans3D(16, -5, 0, 0,0,0)  );

	// Elements:
	const size_t beam0 = problem.insertElement(new CElementBeam_2D_RR(0,1));
	const size_t beam1 = problem.insertElement(new CElementBeam_2D_RR(1,2));
	const size_t beam2 = problem.insertElement(new CElementBeam_2D_AA(0,2));
	const size_t beam3 = problem.insertElement(new CElementBeam_2D_RA(2,3));

	const size_t spring1 = problem.insertElement(new CElementSpring(3,4));

	TParamSet ps;
	ps["K"]  = "1960000";
	ps["E"]  = "2.1e11";
	ps["A"]  = "2.120E-03";
	ps["Iz"] = "3.490E-06";

	for (size_t i=0;i<problem.getNumberOfElements();i++)
		problem.getElement(i)->loadParamsFromSet(ps);

    problem.updateAll();

	// DOF Restrictions:
	problem.insertConstraint( problem.getDOFIndex(0, DX) );
    problem.insertConstraint( problem.getDOFIndex(0, DY) );
    problem.insertConstraint( problem.getDOFIndex(0, RZ) );

	problem.insertConstraint( problem.getDOFIndex(3, DY) );

	problem.insertConstraint( problem.getDOFIndex(4, DX) );

	// External loads:
	problem.addLoadAtDOF( problem.getDOFIndex(3, DX), -2*9810);

	problem.addLoadAtBeam( beam0,  new CLoadConstTemperature( 0 ));
	problem.addLoadAtBeam( beam1,  new CLoadConstTemperature( 0 ));
	problem.addLoadAtBeam( beam3,  new CLoadConstTemperature( 0 ));

	problem.addLoadAtBeam( beam0,  new CLoadDistributedUniform( 9810*0,  0,-1,0 ));
	problem.addLoadAtBeam( beam1,  new CLoadDistributedUniform( 9810*0,  0,-1,0 ));
	problem.addLoadAtBeam( beam3,  new CLoadDistributedUniform( 9810*0,  0,-1,0 ));

	// show all element global matrices:
	if (0)
	{
		for (size_t i=0;i<problem.getNumberOfElements();i++)
		{
			const CElement *e = problem.getElement(i);
			openbeam::aligned_containers<TStiffnessSubmatrix>::vector_t  mats;
			e->getGlobalStiffnessMatrices(mats);

			cout << "ELEMENT " << i << " ------------------------------------\n";
			for (size_t i=0;i<mats.size();i++)
				cout << "K" << mats[i].edge_in << mats[i].edge_out << ":\n" << mats[i].matrix << endl;
		}
	}

#if 0
	{
		const CElement *e = problem.getElement(2);

		if (0)
		{
			TMatrix33 R = e->getGlobalOrientation().getRot();
			cout << "rot:\n" << R << endl;

			cout << "x*y: " << (R.row(0).array()*R.row(1).array()).sum() << endl;
			cout << "x*z: " << (R.row(0).array()*R.row(2).array()).sum() << endl;
			cout << "y*z: " << (R.row(1).array()*R.row(2).array()).sum() << endl;

			mrpt::gui::CDisplayWindow3D  win("test",800,600);


			mrpt::opengl::COpenGLScenePtr &scene = win.get3DSceneAndLock();

			scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
			scene->insert( mrpt::opengl::stock_objects::CornerXYZSimple(0.5,2.0) );

			scene->insert( mrpt::opengl::CSimpleLine::Create(0,0,1, R(0,0),R(0,1),1+R(0,2), 20.0) );
			scene->insert( mrpt::opengl::CSimpleLine::Create(0,0,1, R(1,0),R(1,1),1+R(1,2), 10.0) );
			scene->insert( mrpt::opengl::CSimpleLine::Create(0,0,1, R(2,0),R(2,1),1+R(2,2), 2.0) );

			win.setCameraZoom(4);
			win.unlockAccess3DScene();
			win.forceRepaint();

			win.waitForKey();
		}

		if (0)
		{
			aligned_containers<TStiffnessSubmatrix>::vector_t  sm;
			e->getGlobalStiffnessMatrices(sm);

			for (size_t i=0;i<sm.size();i++)
			{
				const TStiffnessSubmatrix & s = sm[i];
				cout << "i: " << i << endl;
				cout << s.edge_in  << " -> " << s.edge_out << ":\n" << s.matrix << "\n";
			}
		}

	}
#endif

	{
		TBuildProblemInfo info;

		problem.assembleProblem(info);
		//cout << "Kbb:\n" << TDynMatrix(info.K_bb) << endl;
		cout << "Kff:\n" << TDynMatrix(info.K_ff) << endl;
		//cout << "Kbf:\n" << TDynMatrix(info.K_bf) << endl;
		//cout << "Constrains (U_b):\n" << info.U_b << endl;

		cout << "Loads (F_f):\n" << info.F_f << endl;



		CStructureProblem::TStaticSolverOptions  opts;
		//opts.algorithm = ssLLT;
		//opts.nonLinearIterative = true;

		TStaticSolveProblemInfo sInfo;
		problem.solveStatic(sInfo, opts);

		const std::vector<TDoF> & dofs = problem.getProblemDoFs();

		cout << "\nRESULTS:\n------------------------\n";
		cout << "Displacements (U_f):\n";
		size_t nF = sInfo.build_info.free_dof_indices.size();
		for (size_t i=0;i<nF;i++)
		{
			const TDoF &dof = dofs[ sInfo.build_info.free_dof_indices[i] ];

			cout << "U" << dof.node_id+1;
			switch(dof.dof)
			{
			case 0: cout << "x "; break;
			case 1: cout << "y "; break;
			case 2: cout << "z "; break;
			case 3: cout << "Rx"; break;
			case 4: cout << "Ry"; break;
			case 5: cout << "Rz"; break;
			};

			cout << " = "  << sInfo.U_f[i] << endl;
		}

		//cout << "Displacements (U_f):\n" << sInfo.U_f << endl;

		cout << "Reactions (F_b):\n" << sInfo.F_b << endl;
	}



	return 0;
}


int main()
{
	try
	{
		return main_code();
	}catch(std::exception&e) {
		std::cerr << "Exception: " << e.what() << std::endl;
	}
}
