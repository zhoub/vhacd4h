#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
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
    return error();
}

void newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator("hacd",
                                       "HACD",
                                       SOP_HACD::myConstructor,
                                       SOP_HACD::myTemplateList,
                                       1,
                                       1,
                                       0));
}
