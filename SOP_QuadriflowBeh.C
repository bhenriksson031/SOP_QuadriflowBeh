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
    PRM_Name("usedir",	"Use Direction Vector"),
    PRM_Name("dist",	"Distance"),
};

PRM_Template
SOP_QuadriflowBeh::myTemplateList[] = {
    PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
				0, 0, SOP_Node::getGroupSelectButton(
						GA_GROUP_POINT)),
    PRM_Template(PRM_FLT_J,	1, &names[1], PRMzeroDefaults, 0,
				   &PRMscaleRange),
    PRM_Template(PRM_TOGGLE,    1, &names[0]),
    PRM_Template(PRM_ORD,	1, &PRMorientName, 0, &PRMplaneMenu),
    PRM_Template(PRM_DIRECTION, 3, &PRMdirectionName, PRMzaxisDefaults),
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
    // This indicates that this SOP manually manages its data IDs,
    // so that Houdini can identify what attributes may have changed,
    // e.g. to reduce work for the viewport, or other SOPs that
    // check whether data IDs have changed.
    // By default, (i.e. if this line weren't here), all data IDs
    // would be bumped after the SOP cook, to indicate that
    // everything might have changed.
    // If some data IDs don't get bumped properly, the viewport
    // may not update, or SOPs that check data IDs
    // may not cook correctly, so be *very* careful!
    mySopFlags.setManagesDataIDs(true);

    // Make sure to flag that we can supply a guide geometry
    mySopFlags.setNeedGuide1(true);
}

SOP_QuadriflowBeh::~SOP_QuadriflowBeh() {}

bool
SOP_QuadriflowBeh::updateParmsFlags()
{
    bool changed;

    changed  = enableParm(3, !DIRPOP());
    changed |= enableParm(4,  DIRPOP());

    return changed;
}


OP_ERROR
SOP_QuadriflowBeh::cookInputGroups(OP_Context &context, int alone)
{
    // The SOP_Node::cookInputPointGroups() provides a good default
    // implementation for just handling a point selection.
    return cookInputPointGroups(
        context, // This is needed for cooking the group parameter, and cooking the input if alone.
        myGroup, // The group (or NULL) is written to myGroup if not alone.
        alone,   // This is true iff called outside of cookMySop to update handles.
                 // true means the group will be for the input geometry.
                 // false means the group will be for gdp (the working/output geometry).
        true,    // (default) true means to set the selection to the group if not alone and the highlight flag is on.
        0,       // (default) Parameter index of the group field
        -1,      // (default) Parameter index of the group type field (-1 since there isn't one)
        true,    // (default) true means that a pointer to an existing group is okay; false means group is always new.
        false,   // (default) false means new groups should be unordered; true means new groups should be ordered.
        true,    // (default) true means that all new groups should be detached, so not owned by the detail;
                 //           false means that new point and primitive groups on gdp will be owned by gdp.
        0        // (default) Index of the input whose geometry the group will be made for if alone.
    );
}


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

    fpreal now = context.getTime();
	int t1, t2;//quadtime temp
    duplicateSource(0, context);



	if (error() < UT_ERROR_ABORT)
	{
		//QUADR
		int faces = -1;

		//convert to quadriflow mesh
		UT_BoundingBox bbox;
		inputGeo(0, context)->getBBox(&bbox);

		if (field.HouMeshLoader(gdp, bbox) > UT_ERROR_ABORT) {
			//catch load error
			unlockInputs();
			UT_WorkBuffer buf;
			buf.sprintf("Non triangulated mesh, failing to read.");
			addError(SOP_MESSAGE, buf.buffer());
			unlockInputs();
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
			//printf("Estimate Slop...\n");
			t1 = GetCurrentTime64();
			field.EstimateSlope();
			t2 = GetCurrentTime64();
			printf("Use %lf seconds\n", (t2 - t1) * 1e-3);
		}
		printf("Solve for scale...\n");
		//t1 = GetCurrentTime64();
		Optimizer::optimize_scale(field.hierarchy, field.rho, field.flag_adaptive_scale);
		//field.flag_adaptive_scale = 1;
		//t2 = GetCurrentTime64();
		//printf("Use %lf seconds\n", (t2 - t1) * 1e-3);

		printf("Solve for position field...\n");
		//t1 = GetCurrentTime64();
		Optimizer::optimize_positions(field.hierarchy, field.flag_adaptive_scale);
		printf("ComputePositionSingularities Field\n");
		field.ComputePositionSingularities();
		//t2 = GetCurrentTime64();
		//printf("Use %lf seconds\n", (t2 - t1) * 1e-3);
		//t1 = GetCurrentTime64();
		printf("Solve index map...\n");
		field.ComputeIndexMap();
		//t2 = GetCurrentTime64();
		//printf("Indexmap Use %lf seconds\n", (t2 - t1) * 1e-3);

		//printf("Writing the file...\n");
		//if (output_obj.size() < 1) {
		//	assert(0);
		//	// field.OutputMesh((std::string(DATA_PATH) + "/result.obj").c_str());
		//}
		//else {
		//field.OutputMesh(output_obj.c_str());
		//}
		//printf("finish...\n");
		//	field.LoopFace(2);

		// Remove primitives
		// Delete all the primitives, keeping only the points
		gdp->destroyPrimitives(gdp->getPrimitiveRange(), true);

		printf("Dumping geo back to Houdini...\n");
		field.HouMeshDumper(gdp);
		//END QUADR
	}
    // Clears out all the myCur* variables to ensure we have no
    // stray references.  This ensures that if the parameters are
    // evaluated outside of this cook path they don't try to read
    // possibly stale point pointers.
    //resetLocalVarRefs(); //BEH Edit- commented out
    return error();
}

OP_ERROR
SOP_QuadriflowBeh::cookMyGuide1(OP_Context &context)
{
    const int divs = 5;

    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    float now = context.getTime();

    myGuide1->clearAndDestroy();

    float dist = DIST(now);

    float nx = 0;
    float ny = 0;
    float nz = 1;
    if (!DIRPOP())
    {
        switch (ORIENT())
        {
	    case 0 : // XY Plane
	        nx = 0; ny = 0; nz = 1;
	        break;
	    case 1 : // YZ Plane
	        nx = 1; ny = 0; nz = 0;
	        break;
	    case 2 : // XZ Plane
	        nx = 0; ny = 1; nz = 0;
	        break;
	}
    }
    else
    {
        nx = NX(now); ny = NY(now); nz = NZ(now);
    }

    if (error() >= UT_ERROR_ABORT)
        return error();

    UT_Vector3 normal(nx, ny, nz);
    normal.normalize();

    UT_BoundingBox bbox;
    inputGeo(0, context)->getBBox(&bbox);

    float sx = bbox.sizeX();
    float sy = bbox.sizeY();
    float sz = bbox.sizeZ();
    float size = SYSsqrt(sx*sx + sy*sy + sz*sz);

    float cx = normal.x() * dist;
    float cy = normal.y() * dist;
    float cz = normal.z() * dist;

    myGuide1->meshGrid(divs, divs, size, size);

    UT_Vector3 zaxis(0, 0, 1);
    UT_Matrix3 mat3;
    mat3.dihedral(zaxis, normal);
    UT_Matrix4 xform;
    xform = mat3;
    xform.translate(cx, cy, cz);

    myGuide1->transform(xform);

    return error();
}

const char *
SOP_QuadriflowBeh::inputLabel(unsigned) const
{
    return "Geometry to Flatten";
}
