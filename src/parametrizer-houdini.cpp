#include <unordered_map>

#include <Eigen/Core>
#include <vector>

using namespace Eigen;
#include "parametrizer.hpp"

//Houdini
#include <GU/GU_Detail.h>
#include <GA/GA_Detail.h>
#include <GA/GA_Iterator.h>
#include <OP/OP_Error.h>
#include <GA/GA_Stat.h>
#include <GEO/GEO_PrimPoly.h>

//BEH EDIT

void Parametrizer::HouReCook() 
{
	//reinitialize
	
}

int Parametrizer::HouMeshLoader(GU_Detail *gdp, UT_BoundingBox &bbox)
{
	double scale = bbox.sizeMax()*0.5;
	if (scale < .00001) {
		std::cout << "scale is zero!";
		return(14);
	}
	this->normalize_scale = scale;    
	this->normalize_offset = Vector3d(bbox.centerX(), bbox.centerY(), bbox.centerZ());
	GA_RWHandleV3 Phandle(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	GA_Offset ptoff;
	GEO_Primitive *prim;
	int i = 0;
	//set size of matrices
	GA_Range ptrange = gdp->getPointRange();
	GA_Range primrange = gdp->getPrimitiveRange();
	GA_Size npts = gdp->getPointRange().getEntries();
	GA_Size nprims = primrange.getEntries();

	V.resize(3, npts);
	F.resize(3, nprims);
	//std::cout<<"matrix V size: "<< V.size() << "\n";
	//std::cout << "matrix F size: " << F.size() <<"\n";
	GA_FOR_ALL_PTOFF(gdp, ptoff) {
		UT_Vector3 Pvalue = (Phandle.get(ptoff) - bbox.center() ) / scale ;
		V(0, i) = Pvalue.x();
		V(1, i) = Pvalue.y();
		V(2, i) = Pvalue.z();
		i++;
	}

	int j = 0;
	GA_FOR_ALL_PRIMITIVES(gdp, prim) {

		GA_OffsetListRef prim_vrts = gdp->getPrimitiveVertexList(prim->getMapOffset());
		
		if (prim_vrts.entries() != 3) {
			return(14); //Bad input code
		}
		F(0, j) = gdp->vertexPoint(prim_vrts(0));
		F(1, j) = gdp->vertexPoint(prim_vrts(1));
		F(2, j) = gdp->vertexPoint(prim_vrts(2));
		j++;
	}
	return(0);
}

void Parametrizer::HouMeshDumper(GU_Detail *gdp) {
	//create all points
	std::vector<GA_Offset> added_pts;
	UT_Vector3 offs = UT_Vector3(this->normalize_offset.x(), this->normalize_offset.y(), this->normalize_offset.z());
	for (int i = 0; i < O_compact.size(); ++i) {

		GA_Offset ptoff = gdp->appendPoint();
		added_pts.push_back(ptoff);

		UT_Vector3 PValue = UT_Vector3(O_compact[i].x(), O_compact[i].y(), O_compact[i].z() ) * this->normalize_scale + offs;
		gdp->setPos3(ptoff, PValue);
	}

	for (int i = 0; i < F_compact.size(); ++i)
	{

		GA_Offset pt0 = added_pts[F_compact[i][0]];
		GA_Offset pt1 = added_pts[F_compact[i][1]];
		GA_Offset pt2 = added_pts[F_compact[i][2]];
		GA_Offset pt3 = added_pts[F_compact[i][3]];
		GEO_PrimPoly *poly = (GEO_PrimPoly *)gdp->appendPrimitive(GA_PRIMPOLY);
		poly->appendVertex(pt0);
		poly->appendVertex(pt1);
		poly->appendVertex(pt2);
		poly->appendVertex(pt3);
		poly->close();
	}
	//loop over matrix
	//create primitives
	//loop over matrix 

	//for (int idx = 0; idx < indices.size(); idx += 3)
	//{
	//	const EarCutHoudiniPoint& houdini_point_0 = gdp_points[indices[idx + 0]];
	//	const EarCutHoudiniPoint& houdini_point_1 = gdp_points[indices[idx + 1]];
	//	const EarCutHoudiniPoint& houdini_point_2 = gdp_points[indices[idx + 2]];

	//	GU_PrimPoly* poly = GU_PrimPoly::build(gdp, 0, GU_POLY_CLOSED);
	//	poly->appendVertex(houdini_point_0.point_offset);
	//	poly->appendVertex(houdini_point_1.point_offset);
	//	poly->appendVertex(houdini_point_2.point_offset);
	//	poly->close();
	//}

	//gdp->getAttributes().bumpAllDataIds(GA_ATTRIB_PRIMITIVE);  //UNCOMMENTED AS NODE IS SET TO NOT MANAGE ITS DATA IDS
	//gdp->getPrimitiveList().bumpDataId();


	//std::ofstream os(obj_name);
	//for (int i = 0; i < O_compact.size(); ++i) {
	//	auto t = O_compact[i] * this->normalize_scale + this->normalize_offset;
	//	os << "v " << t[0] << " " << t[1] << " " << t[2] << "\n";
	//}
	//for (int i = 0; i < F_compact.size(); ++i) {
	//	os << "f " << F_compact[i][0] + 1 << " " << F_compact[i][1] + 1
	//		<< " " << F_compact[i][2] + 1 << " " << F_compact[i][3] + 1
	//		<< "\n";
	//}
	//os.close();

}