/*
 * Copyright (c) 2018
 *	Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------
 * The Flatten SOP.  This SOP Flattens the geometry onto a plane.
 */

#include "SOP_QuadriflowBeh.h" 

//from houdini example code
#include <SOP/SOP_Guide.h>
#include <GU/GU_Detail.h>
#include <GA/GA_Detail.h>
#include <GA/GA_Iterator.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Vector3.h>
#include <SYS/SYS_Math.h>
#include <stddef.h>


using namespace HDK_Beh;

void
newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator(
        "quadriflow_beh",
        "Quadriflow",
        SOP_QuadriflowBeh::myConstructor,
        SOP_QuadriflowBeh::myTemplateList,
        1,
        1,
        NULL));
}

static PRM_Name names[] = {
    PRM_Name("sharp",	"Preserve Sharp"),
    PRM_Name("adaptive", "Adaptive Scale"),
	PRM_Name("mcf",	"Minimum Cost Flow"),
	PRM_Name("sat",	"Aggresive Sat (disabled)"),
	PRM_Name("faces",	"Faces"),
};

PRM_Template
SOP_QuadriflowBeh::myTemplateList[] = {
	PRM_Template(PRM_TOGGLE,    1, &names[0], PRMzeroDefaults),
	PRM_Template(PRM_TOGGLE,    1, &names[1], PRMzeroDefaults),
	PRM_Template(PRM_TOGGLE,    1, &names[2], PRMzeroDefaults),
	PRM_Template(PRM_TOGGLE,    1, &names[3], PRMzeroDefaults),
    PRM_Template(PRM_INT,	1, &names[4], PRMnegoneDefaults, 0,
				   &PRMfullunituiRange),
    PRM_Template(),
};


OP_Node *
SOP_QuadriflowBeh::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_QuadriflowBeh(net, name, op);
}

SOP_QuadriflowBeh::SOP_QuadriflowBeh(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op), myGroup(NULL)
{
	//mySopFlags.setManagesDataIDs(true);  //manually set data ids
}

SOP_QuadriflowBeh::~SOP_QuadriflowBeh() {}

OP_ERROR
SOP_QuadriflowBeh::cookMySop(OP_Context &context)
{
	Parametrizer field; //quadriflow object

	UT_AutoInterrupt progress("Init Quadriflow Node");
    // We must lock our inputs before we try to access their geometry.
    // OP_AutoLockInputs will automatically unlock our inputs when we return.
    // NOTE: Don't call unlockInputs yourself when using this!
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    fpreal t = context.getTime();
	int t1, t2;//quadtime temp
    duplicateSource(0, context);
	UT_BoundingBox bbox;
	inputGeo(0, context)->getBBox(&bbox);

	if (error() < UT_ERROR_ABORT)
	{
		//QUADR
		int  faces = FACES(t);
		field.flag_preserve_sharp = SHARP(t);
		field.flag_adaptive_scale  = ADAPTIVE(t);
		field.flag_minimum_cost_flow = MCF(t);
		field.flag_aggresive_sat = 0;// SAT(t);
		printf("field.flag_adaptive_scale  = %d", field.flag_adaptive_scale);
		printf("ADAPTIVE(t)  = %d", ADAPTIVE(t) );
		//convert to quadriflow mesh

		printf("Load Hou Mesh\n");
		if (field.HouMeshLoader(gdp, bbox) > UT_ERROR_ABORT) {
			//catch load error
			UT_WorkBuffer buf;
			buf.sprintf("Non triangulated mesh, failing to read.");
			addError(SOP_MESSAGE, buf.buffer());
			return error();
		}
		//init quadflow object
		printf("Initialize...\n");
		field.Initialize(faces);


		printf("Solve Orientation Field...\n");
		Optimizer::optimize_orientations(field.hierarchy);  //failing

		printf("ComputeOrientationSingularities Field\n");
		field.ComputeOrientationSingularities();

		if (field.flag_adaptive_scale == 1) {
			printf("Estimate Slop...\n");
			t1 = GetCurrentTime64();
			field.EstimateSlope();
			t2 = GetCurrentTime64();
			printf("Use %lf seconds\n", (t2 - t1) * 1e-3);
		}
		printf("Solve for scale...\n");
		t1 = GetCurrentTime64();
		Optimizer::optimize_scale(field.hierarchy, field.rho, field.flag_adaptive_scale);
		field.flag_adaptive_scale = 1;
		t2 = GetCurrentTime64();
		printf("Use %lf seconds\n", (t2 - t1) * 1e-3);

		printf("Solve for position field...\n");
		t1 = GetCurrentTime64();
		Optimizer::optimize_positions(field.hierarchy, field.flag_adaptive_scale);
		printf("ComputePositionSingularities Field\n");
		field.ComputePositionSingularities();
		t2 = GetCurrentTime64();
		printf("Use %lf seconds\n", (t2 - t1) * 1e-3);
		//t1 = GetCurrentTime64();
		printf("Solve index map...\n");
		field.ComputeIndexMap();

		//output
		// Remove primitives
		// Delete all the primitives, keeping only the points
		//gdp->destroyPrimitives(gdp->getPrimitiveRange(), true);

		
		printf("Dumping geo back to Houdini...\n");
		gdp->clearAndDestroy();
		field.HouMeshDumper(gdp);
		//END QUADR
	}
    return error();
}

const char *
SOP_QuadriflowBeh::inputLabel(unsigned) const
{
    return "Triangulated mesh to Quads";
}
