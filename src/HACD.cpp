#include <iomanip>

#include <GEO/GEO_PrimPoly.h>
#include <GEO/GEO_PrimPolySoup.h>
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

class StdCallback : public VHACD::IVHACD::IUserCallback
{
public:
    StdCallback()
    {
    }
    ~StdCallback()
    {
    }

    void Update(const double overallProgress,
                const double stageProgress,
                const double operationProgress,
                const char * const stage,
                const char * const operation)
    {
        std::cout << std::setfill(' ') << std::setw(3) << (int)(overallProgress  +0.5) << "% "
             << "[ " << stage     << " " << std::setfill(' ') << std::setw(3) << (int)(stageProgress    +0.5) << "% ] "
                     << operation << " " << std::setfill(' ') << std::setw(3) << (int)(operationProgress+0.5) << "%" << std::endl;
    };
};

class StdLogger : public VHACD::IVHACD::IUserLogger
{
public:
    StdLogger()
    {
    }

    ~StdLogger()
    {
    }

    void Log(const char * const msg)
    {
        std::cout << msg;
    }
};

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

    void evalParams(fpreal time);

    VHACD::IVHACD::Parameters myHacdParams;

    static StdCallback myCallback;
    static StdLogger myLogger;
};

StdCallback SOP_HACD::myCallback;
StdLogger SOP_HACD::myLogger;

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
    SOP_Node(net, name, op),
    myHacdParams()
{
    myHacdParams.m_oclAcceleration = false;
    myHacdParams.m_logger = &myLogger;
    myHacdParams.m_callback = &myCallback;
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

    std::vector<float> pointVec;
    pointVec.reserve(gdp->getNumPoints() * 3);

    const GA_Attribute * attrP = gdp->getP();
    const GA_Detail & detailP = attrP->getDetail();
    const GA_Range rangeP = detailP.getPointRange();

    UT_ValArray<UT_Vector3> arrayP;
        detailP.getPos3AsArray(rangeP, arrayP);
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
    evalParams(context.getTime());
    VHACD::IVHACD * interfaceVHACD = VHACD::CreateVHACD();

    bool success = interfaceVHACD->Compute(&pointVec[0], 3, pointVec.size() / 3, &idxVec[0], 3, idxVec.size() / 3, myHacdParams);
    if (success && interfaceVHACD->GetNConvexHulls())
    {
        for (unsigned int i = 0; i < interfaceVHACD->GetNConvexHulls(); ++ i)
        {
            VHACD::IVHACD::ConvexHull ch;
            ch.m_nPoints = 0;
            ch.m_points = NULL;
            ch.m_nTriangles = 0;
            ch.m_triangles = NULL;
            interfaceVHACD->GetConvexHull(i, ch);
            if(! ch.m_nPoints || ! ch.m_nTriangles)
            {
                continue;
            }

            //
            std::vector<GA_Offset> pOffsets;
            pOffsets.reserve(ch.m_nPoints);
            for (unsigned int p = 0; p < ch.m_nPoints * 3; p += 3)
            {
                GA_Offset p_offset = gdp->appendPointOffset();
                pOffsets.push_back(p_offset);
                const UT_Vector3 point(ch.m_points[p], ch.m_points[p + 1], ch.m_points[p + 2]);
                gdp->setPos3(p_offset, point);
            }

            for (unsigned int t = 0; t < ch.m_nTriangles * 3; t += 3)
            {
                GEO_PrimPoly *tri = static_cast<GEO_PrimPoly *>(gdp->appendPrimitive(GEO_PRIMPOLY));
                tri->setSize(0);
                tri->appendVertex(pOffsets[ch.m_triangles[t + 0]]);
                tri->appendVertex(pOffsets[ch.m_triangles[t + 1]]);
                tri->appendVertex(pOffsets[ch.m_triangles[t + 2]]);
                tri->close();
            }
        }
    }
    else
    {
        addError(SOP_MESSAGE, "Computation failed");
    }

    interfaceVHACD->Clean();
    interfaceVHACD->Release();

    return error();
}

void SOP_HACD::evalParams(fpreal time)
{
    myHacdParams.m_resolution = evalInt(resolutionName.getToken(), 0, time);
    myHacdParams.m_depth = evalInt(depthName.getToken(), 0, time);
    myHacdParams.m_concavity = evalFloat(concavityName.getToken(), 0, time);
    myHacdParams.m_delta = evalFloat(deltaName.getToken(), 0, time);
    myHacdParams.m_planeDownsampling = evalInt(planeDownsamplingName.getToken(), 0, time);
    myHacdParams.m_convexhullDownsampling = evalInt(convexhullDownsamplingName.getToken(), 0, time);
    myHacdParams.m_alpha = evalFloat(alphaName.getToken(),0, time);
    myHacdParams.m_beta = evalFloat(betaName.getToken(), 0, time);
    myHacdParams.m_gamma = evalFloat(gammaName.getToken(), 0, time);
    myHacdParams.m_pca = evalInt(pcaName.getToken(), 0, time);
    myHacdParams.m_mode = evalInt(modeName.getToken(), 0, time);
    myHacdParams.m_maxNumVerticesPerCH = evalInt(maxNumVerticesPerCHName.getToken(), 0, time);
    myHacdParams.m_minVolumePerCH = evalFloat(minVolumePerChName.getToken(), 0, time);
    myHacdParams.m_convexhullApproximation = evalInt(convexhullApproximationName.getToken(), 0, time);
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
