/*
 *----------------------------------------------------------------------------
 * Quadriflow SOP by bhenriksson@gmail.com
 */


#ifndef __SOP_QuadriflowBeh_h__
#define __SOP_QuadriflowBeh_h__

#include <SOP/SOP_Node.h>

 //Beh edits
#include <Eigen/Core>
#include <Eigen/Dense>
#include "src/config.hpp">
#include <src/parametrizer.hpp>
#include <src/optimizer.hpp>


namespace HDK_Beh {
} // End HDK_Beh namespace


class SOP_QuadriflowBeh : public SOP_Node
{
public:
	SOP_QuadriflowBeh(OP_Network *net, const char *name, OP_Operator *op);
    virtual ~SOP_QuadriflowBeh();

    static PRM_Template		 myTemplateList[];
    static OP_Node		*myConstructor(OP_Network*, const char *,
							    OP_Operator *);

protected:

    virtual const char          *inputLabel(unsigned idx) const;

    /// Method to cook geometry for the SOP
    virtual OP_ERROR		 cookMySop(OP_Context &context);

private:

    int	FACES(fpreal t)		{ return evalInt("faces", 0, t); }
	int SHARP(fpreal t) { return evalInt("sharp", 0, t); }
	int ADAPTIVE(fpreal t) { return evalInt("adaptive", 0, t); }
	int MCF(fpreal t) { return evalInt("mcf", 0, t); }
	int SAT(fpreal t) { return evalInt("sat", 0, t); }

    /// This is the group of geometry to be manipulated by this SOP and cooked
    /// by the method "cookInputGroups".
    const GA_PointGroup *myGroup;


};


#endif
