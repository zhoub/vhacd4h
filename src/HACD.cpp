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

#include <VHACD.h>
#include <vhacdTimer.h>
#include <vhacdVolume.h>
#include <vhacdVHACD.h>

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

    VHACD::IVHACD::Parameters myHacdParams;
};

//
static PRM_Name resolutionName("resolution", "Resolution");
static PRM_Default resolutionDefault(100000);

static PRM_Name depthName("depth", "Depth");
static PRM_Default depthDefault(20);

static PRM_Name concavityName("concavity", "Concavity");
static PRM_Default concavityDefault(0.001);

static PRM_Name deltaName("delta", "Delta");
static PRM_Default deltaDefault(0.05);

static PRM_Name planeDownsamplingName("planeDownsamping", "Plane Downsampling");
static PRM_Default planeDownsamplingDefault(4);

static PRM_Name convexhullDownsamplingName("convexhullDownsampling", "Convex Hull Downsampling");
static PRM_Default convexhullDownsamplingDefault(4);

static PRM_Name alphaName("alpha", "Alpha");
static PRM_Default alphaDefault(0.05);

static PRM_Name betaName("beta", "Beta");
static PRM_Default betaDefault(0.05);

static PRM_Name gammaName("gamma", "Gamma");
static PRM_Default gammaDefault(0.0005);

static PRM_Name pcaName("pca", "PCA");
static PRM_Default pcaDefault(0);

static PRM_Name modeName("mode", "Mode");
static PRM_Name modeChoices[] =
{
    PRM_Name("voxel", "Voxel"),
    PRM_Name("tetrahedron", "Tetrahedron"),
    PRM_Name()
};
static PRM_ChoiceList modeMenu(PRM_CHOICELIST_SINGLE, modeChoices);
static PRM_Default modeDefault(0, "voxel");

static PRM_Name maxNumVerticesPerCHName("maxNumVerticesPerCH", "Max Num. Vertices Per Convex Hull");
static PRM_Default maxNumVerticesPerCHDefault(64);

static PRM_Name minVolumePerChName("minVolumePerCh", "Min Volume Per Ch");
static PRM_Default minVolumePerChDefault(0.0001);

static PRM_Name convexhullApproximationName("convexhullApproximation", "Convex Hull Approximation");
static PRM_Default convexhullApproximationDefault(true);

PRM_Template SOP_HACD::myTemplateList[] =
{
    PRM_Template(PRM_INT, PRM_Template::PRM_EXPORT_MED, 1, &resolutionName, &resolutionDefault),
    PRM_Template(PRM_INT, PRM_Template::PRM_EXPORT_MED, 1, &depthName, &depthDefault),
    PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MED, 1, &concavityName, &concavityDefault),
    PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MED, 1, &deltaName, &deltaDefault),
    PRM_Template(PRM_INT, PRM_Template::PRM_EXPORT_MED, 1, &planeDownsamplingName, &planeDownsamplingDefault),
    PRM_Template(PRM_INT, PRM_Template::PRM_EXPORT_MED, 1, &convexhullDownsamplingName, &convexhullDownsamplingDefault),
    PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MED, 1, &alphaName, &alphaDefault),
    PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MED, 1, &betaName, &betaDefault),
    PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MED, 1, &gammaName, &gammaDefault),
    PRM_Template(PRM_INT, PRM_Template::PRM_EXPORT_MED, 1, &pcaName, &pcaDefault),
    PRM_Template(PRM_ORD, PRM_Template::PRM_EXPORT_MED, &modeName, &modeDefault, &modeMenu),
    PRM_Template(PRM_INT, PRM_Template::PRM_EXPORT_MED, 1, &maxNumVerticesPerCHName, &maxNumVerticesPerCHDefault),
    PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MED, 1, &minVolumePerChName, &minVolumePerChDefault),
    PRM_Template(PRM_TOGGLE, PRM_Template::PRM_EXPORT_MED, &convexhullApproximationName, &convexhullApproximationDefault),
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
    myHacdParams.m_oclAcceleration = false;
}

SOP_HACD::~SOP_HACD()
{
}

OP_ERROR SOP_HACD::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    //
    duplicateSource(0, context);

    std::vector<double> pointVec;
    pointVec.reserve(gdp->getNumPoints() * 3);

    const GA_Attribute * attrP = gdp->getP();
    const GA_Detail & detailP = attrP->getDetail();
    const GA_Range rangeP = detailP.getPointRange();

    UT_ValArray<UT_Vector3> arrayP;
    for (UT_ValArray<UT_Vector3>::const_iterator itr = arrayP.begin(); itr != arrayP.end(); ++ itr)
    {
        pointVec.push_back(itr->x());
        pointVec.push_back(itr->y());
        pointVec.push_back(itr->z());
    }

    //
    std::vector<int> idxVec;
    idxVec.reserve(gdp->getNumPrimitives() * 3);

    GEO_Primitive * prim = NULL;
    GA_FOR_ALL_PRIMITIVES(gdp, prim)
    {
        const GA_Size vtxCount = prim->getVertexCount();
        if (vtxCount != 3)
        {
            addError(SOP_MESSAGE, "Not triangulated");
            return error();
        }
        for (GA_Size i = 0; i < vtxCount; ++ i)
        {
            idxVec.push_back(prim->getPointOffset(i));
        }
    }

    gdp->clear();

    //
    VHACD::IVHACD * interfaceVHACD = VHACD::CreateVHACD();

    bool success = interfaceVHACD->Compute(&pointVec[0], 3, pointVec.size() / 3, &idxVec[0], 3, idxVec.size() / 3, myHacdParams);
    if (success)
    {
        std::cout << interfaceVHACD->GetNConvexHulls() << std::endl;
    }
    else
    {
        addError(SOP_MESSAGE, "Computed failed");
    }

    interfaceVHACD->Clean();
    interfaceVHACD->Release();

    //
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
                                       1,
                                       1,
                                       NULL,
                                       OP_FLAG_GENERATOR));
}
