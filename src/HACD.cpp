#include <GEO/GEO_PrimPoly.h>
#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
#include <GA/GA_ElementGroup.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Director.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <UT/UT_DSOVersion.h>
#include <SOP/SOP_Node.h>
#include <SYS/SYS_Math.h>

class SOP_HACD : public SOP_Node
{
public:

    static PRM_Template myTemplateList[];
    static OP_Node *myConstructor(OP_Network *net, const char *name, OP_Operator *op);

    SOP_HACD(OP_Network *net, const char *name, OP_Operator *op);

    virtual ~SOP_HACD();

protected:

    virtual OP_ERROR cookMySop(OP_Context &context);

private:
};

PRM_Template SOP_HACD::myTemplateList[] =
{
    PRM_Template()
};

OP_Node * SOP_HACD::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_HACD(net, name, op);
}

SOP_HACD::SOP_HACD(OP_Network *net, const char *name, OP_Operator *op)
    :
    SOP_Node(net, name, op)
{
}

SOP_HACD::~SOP_HACD()
{
}

OP_ERROR SOP_HACD::cookMySop(OP_Context &context)
{
    /*
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    //
    duplicateSource(0, context);
    */

    GEO_PrimPoly *quad = GEO_PrimPoly::build(gdp, 4, GU_POLY_CLOSED);
    GA_Offset offset = -1;

    offset = quad->getPointOffset(0);
    gdp->setPos3(offset, UT_Vector3(-1, 0, -1));

    offset = quad->getPointOffset(1);
    gdp->setPos3(offset, UT_Vector3(1, 0, -1));

    offset = quad->getPointOffset(2);
    gdp->setPos3(offset, UT_Vector3(1, 0, 1));

    offset = quad->getPointOffset(3);
    gdp->setPos3(offset, UT_Vector3(-1, 0, 1));

    //
    GA_PrimitiveGroup *quadGroup = gdp->newPrimitiveGroup("quad");
    quadGroup->addIndex(GA_Index(0));
    quadGroup->addIndex(GA_Index(1));
    quadGroup->addIndex(GA_Index(2));
    quadGroup->addIndex(GA_Index(3));

    return error();
}

void newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator("hacd",
                                       "HACD",
                                       SOP_HACD::myConstructor,
                                       SOP_HACD::myTemplateList,
                                       0,
                                       0,
                                       NULL,
                                       OP_FLAG_GENERATOR));
}
