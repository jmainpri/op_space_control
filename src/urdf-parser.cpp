#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

#include <fstream>
#include <utils/stringutils.h>
#include <string>
#include <fstream>
#include <boost/shared_ptr.hpp>

#include "urdf-parser.h"

using std::cout;
using std::endl;

namespace op_space_control // for linking with robotsim
{
class URDFLinkNode
{
public:
    URDFLinkNode(boost::shared_ptr<urdf::Link>& link, int index, int index_parent);
    void GetTransformations();
    void GetGeometryProperty();
    void GetJoint();
    boost::shared_ptr<urdf::Link> link;
    int index;
    int index_parent;
    RigidTransform T_link_to_inertia;
    RigidTransform T_link_to_inertia_inverse;
    RigidTransform T_link_to_visgeom;
    RigidTransform T_link_to_colgeom;
    RigidTransform T_parent;
    Vector3 axis;
    bool geomPrimitive;
    std::string geomName;
    Matrix4 geomScale;
    urdf::Joint* joint;
};

class URDFConverter
{
public:
    static int GetLinkIndexfromName(std::string name, const std::vector<std::string> linknames);
    static RobotJoint::Type jointType_URDF2ROB(int );
    static void DFSLinkTree( URDFLinkNode& root, std::vector<URDFLinkNode>& linkNodes);
    static void setJointforNodes(std::vector< boost::shared_ptr<urdf::Joint> >& joints, std::vector<URDFLinkNode>& linkNodes);
    static Math3D::Matrix3 convertInertial( urdf::Inertial& I);
    static void QuatToRotationMat(const Vector4& aa, Matrix3& mat);
    static void processTParentTransformations(std::vector<URDFLinkNode>& linkNodes);
    static void ConvertWrltoTri(std::string filename);
    //	static void ScalePrimitiveGeom(string infilename, string outfilename);

    //The location of primitive_mesh must be provided.
    static std::string primitive_mesh_path;
};

std::string URDFConverter::primitive_mesh_path("data/objects/urdf_primitives/");

//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------

URDFLinkNode::URDFLinkNode(boost::shared_ptr<urdf::Link>& _link, int _index, int
                           _index_parent)
{
    link = _link;
    index = _index;
    index_parent = _index_parent;
    T_link_to_inertia.setIdentity();
    T_link_to_inertia_inverse.setIdentity();
    T_link_to_visgeom.setIdentity();
    T_link_to_colgeom.setIdentity();
    T_parent.setIdentity();
    joint = NULL;
    axis.set(0,0,1);

    GetTransformations();
    //GetGeometryProperty();
    return;
}

void URDFLinkNode::GetGeometryProperty()
{
    if(!link){
        cout<<"link is NULL!"<<endl;
        return;
    }
    geomScale.setIdentity();
    geomPrimitive = false;
    if(link->collision && link->collision->geometry)
    {
        if(link->collision->geometry->type == link->collision->geometry->BOX)
        {
            geomPrimitive = true;
            geomName = URDFConverter::primitive_mesh_path + "box_ori_center.tri";
            boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(link->collision->geometry);
            geomScale(0,0) = box->dim.x;
            geomScale(1,1) = box->dim.y;
            geomScale(2,2) = box->dim.z;

        }
        else if(link->collision->geometry->type == link->collision->geometry->CYLINDER)
        {
            geomPrimitive = true;
            geomName = URDFConverter::primitive_mesh_path + "cylinder_ori_center.tri";
            boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>(link->collision->geometry);
            geomScale(0,0) = cylinder->radius;
            geomScale(1,1) = cylinder->radius;
            geomScale(2,2) = cylinder->length;
        }
        if(link->collision->geometry->type == link->collision->geometry->SPHERE)
        {
            geomPrimitive = true;
            geomName = URDFConverter::primitive_mesh_path + "sphere_ori_center.tri";
            boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(link->collision->geometry);
            geomScale(0,0) = sphere->radius;
            geomScale(1,1) = sphere->radius;
            geomScale(2,2) = sphere->radius;
        }
        if(link->collision->geometry->type == link->collision->geometry->MESH)
        {
            boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(link->collision->geometry);
            geomName = mesh->filename.c_str();
            geomScale(0,0) = mesh->scale.x;
            geomScale(1,1) = mesh->scale.y;
            geomScale(2,2) = mesh->scale.z;
        }
    }
    else
    {
        geomName = "";
        geomScale.setIdentity();
    }
}

void URDFLinkNode::GetTransformations()
{
    if(!link)
    {
        cout<<"link is NULL!"<<endl;
        return;
    }

    urdf::Vector3 pos;
    urdf::Rotation rotation;
    Vector4 quat;

    if( link->inertial )
    {
        pos = link->inertial->origin.position;
        rotation = link->inertial->origin.rotation;
        quat.set(rotation.x, rotation.y, rotation.z, rotation.w);
        URDFConverter::QuatToRotationMat(quat, T_link_to_inertia.R);
        T_link_to_inertia.t.set(pos.x, pos.y, pos.z);
        T_link_to_inertia_inverse.setInverse(T_link_to_inertia);
    }

    if( link->collision )
    {
        pos = link->collision->origin.position;
        rotation = link->collision->origin.rotation;
        quat.set(rotation.x, rotation.y, rotation.z, rotation.w);
        URDFConverter::QuatToRotationMat(quat, T_link_to_colgeom.R);
        T_link_to_colgeom.t.set(pos.x, pos.y, pos.z);
    }

    if( link->visual )
    {
        pos = link->visual->origin.position;
        rotation = link->visual->origin.rotation;
        quat.set(rotation.x, rotation.y, rotation.z, rotation.w);
        URDFConverter::QuatToRotationMat(quat, T_link_to_visgeom.R);
        T_link_to_visgeom.t.set(pos.x, pos.y, pos.z);
    }
}

//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------

int URDFConverter::GetLinkIndexfromName(std::string name, const std::vector<std::string> linknames){
    int link_index = -1;
    for (int i = 0; i < linknames.size(); i++) {
        if ( strcmp(name.c_str(), linknames[i].c_str()) == 0) {
            link_index = i;
            break;
        }
    }
    return link_index;
}

RobotJoint::Type URDFConverter::jointType_URDF2ROB(int type) {
    //ROB { Weld, Normal, Spin, Floating, FloatingPlanar, BallAndSocket, Closed };
    //URDF {UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED };

    RobotJoint::Type robtype = RobotJoint::Weld;
    if (type == urdf::Joint::REVOLUTE)
        robtype = RobotJoint::Normal;
    if (type == urdf::Joint::CONTINUOUS)
        robtype = RobotJoint::Spin;
    else if (type == urdf::Joint::FIXED)
        robtype = RobotJoint::Weld;
    else if (type == urdf::Joint::FLOATING)
        robtype = RobotJoint::Floating;
    else if (type == urdf::Joint::PLANAR)
        robtype = RobotJoint::FloatingPlanar;
    else if (type == urdf::Joint::PRISMATIC) {
        robtype = RobotJoint::Normal;
    }
    return robtype;
}

void URDFConverter::DFSLinkTree(URDFLinkNode& root, std::vector<URDFLinkNode>& linkNodes)
{
    linkNodes.push_back( root);

    //Parent has index
    for (int i = 0; i < root.link->child_links.size(); i++)
    {
        URDFLinkNode child( root.link->child_links[i], linkNodes.size(), root.index);
        DFSLinkTree( child, linkNodes);
    }
}

void URDFConverter::setJointforNodes(std::vector< boost::shared_ptr<urdf::Joint> >& joints, std::vector<URDFLinkNode>& linkNodes)
{
    for(int i = 0; i < linkNodes.size(); i++)
    {
        std::string linkname = linkNodes[i].link->name;
        linkNodes[i].joint = 0;
        for (int j = 0; j < joints.size(); j++)
        {
            boost::shared_ptr<urdf::Joint> joint = joints[j];
            //find link index of the joint
            std::string joint_name = joint->child_link_name;
            if(0 == strcmp(joint_name.c_str(), linkname.c_str())){
                linkNodes[i].joint = joint.get();
                break;
            }
        }
    }
}

void URDFConverter::QuatToRotationMat(const Vector4& aa, Matrix3& mat)
{
    double y,z,w,x;
    x = aa.x;
    y = aa.y;
    z = aa.z;
    w = aa.w;

    mat(0,0) = 1 - 2*y*y - 2*z*z;
    mat(0,1) = 2*x*y + 2*z*w;
    mat(0,2) = 2*x*z - 2*w*y;
    mat(1,0) = 2*y*x - 2*z*w;
    mat(1,1) = 1 - 2*x*x - 2*z*z;
    mat(1,2) = 2*z*y + 2*x*w;
    mat(2,0) = 2*x*z + 2*w*y;
    mat(2,1) = 2*z*y - 2*x*w;
    mat(2,2) = 1 - 2*x*x - 2*y*y;
}

Math3D::Matrix3 URDFConverter::convertInertial(urdf::Inertial& I)
{
    Math3D::Matrix3 m;
    m(0, 0) = I.ixx;
    m(0, 1) = m(1, 0) = I.ixy;
    m(0, 2) = m(2, 0) = I.ixz;
    m(1, 1) = I.iyy;
    m(1, 2) = m(2, 1) = I.iyz;
    m(2, 2) = I.izz;
    return m;
}

void URDFConverter::processTParentTransformations(std::vector<URDFLinkNode>& linkNodes)
{
    for(int i = 0; i < linkNodes.size(); i++)
    {
        RigidTransform T0, T1, T2;
        T0.setIdentity();
        Vector3 tmpaxis;
        if(linkNodes[i].joint){
            urdf::Pose pose = linkNodes[i].joint->parent_to_joint_origin_transform;
            T0.t.set(pose.position.x, pose.position.y, pose.position.z);
            Vector4 quat(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
            QuatToRotationMat(quat, T0.R);

            tmpaxis.set( linkNodes[i].joint->axis.x, linkNodes[i].joint->axis.y, linkNodes[i].joint->axis.z);
            if(tmpaxis.norm() > 0){
                linkNodes[i].axis.set( linkNodes[i].joint->axis.x, linkNodes[i].joint->axis.y, linkNodes[i].joint->axis.z);
            }

        }
        linkNodes[i].T_parent.set(T0);

        RigidTransform G0, G1;
        G0.mul(linkNodes[i].T_link_to_inertia_inverse, linkNodes[i].T_link_to_colgeom);
        G1.mul(linkNodes[i].T_link_to_colgeom, linkNodes[i].geomScale);
        linkNodes[i].geomScale.set(G1);
    }
}

//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------

//std::string GetFilePath(const std::string& str)
//{
//    size_t pos = str.rfind('\\');
//    if(pos == std::string::npos){
//        pos = str.rfind('/');
//        if(pos == std::string::npos){
//            return "";
//        }
//    }
//    return str.substr(0,pos+1);
//}

bool Robot::load_urdf( std::string filename )
{
//    std::string s( filename );
//    std::string path = GetFilePath(s);

    // Read the urdf into a string
    std::string xml_model_string;
    std::fstream xml_file( filename.c_str(), std::fstream::in );

    while( xml_file.good() )
    {
        std::string line;
        std::getline( xml_file, line );
        xml_model_string += ( line + "\n" );
    }

    xml_file.close();

    // Get content from the Willow Garage parser
    boost::shared_ptr<urdf::ModelInterface> parser = urdf::parseURDF( xml_model_string );

    cout << "URDF File contains robot : "<< parser->getName() << endl;

    //links_size: URDF 36, ROB 41 with 5 extra DOFS for base
    int links_size = parser->links_.size() + 5;
    //joints_size: URDF 35, ROB 36 with 1 extra for base
    int joints_size = parser->joints_.size() + 1;
    //drivers_size: should be joints_size - 1 in ROB file

    if( joints_size != links_size - 5 )
    {
        cout << "joint size:" << joints_size << " and link size:" << links_size
             << " are Not match!" << endl;
        return false;
    }

    cout << "Link size:" << links_size << endl;
    cout << "Joint size:" << joints_size << endl;
    cout << this << endl;

    // Feed the information from parser to required vectors in ROB format
    // The following vectors have the same dimension of the links vector
    this->Initialize(links_size);

    this->links.resize(links_size);
    this->parents.resize(links_size);
    this->linkNames.resize(links_size);
    //this->geometry.resize(links_size); // TODO
    this->q.resize(links_size); // Weird error segfault
    this->q.setZero();
    this->qMin.resize(links_size);
    this->qMax.resize(links_size);
    this->accMax.resize(links_size);
    this->torqueMax.resize(links_size);
    this->powerMax.resize(links_size); //TODO
    this->velMax.resize(links_size);
    this->velMin.resize(links_size);

    // The following vectors have different dimensions
    this->joints.resize(joints_size);

    // floating base, the first 6 degrees
    for (int i=0; i<6; i++)
    {
        this->parents[i] = i - 1;
        std::stringstream ss;
        ss << i;
        std::string tmp;
        ss >> tmp;
        this->linkNames[i] = "base" + tmp;
        this->accMax[i] = Inf;
        this->qMax[i] = Inf;
        this->qMin[i] = -Inf;
        this->velMax[i] = Inf;
        this->velMin[i] = -Inf;
        this->torqueMax[i] = 0;
        this->powerMax[i] = Inf;
    }

    for (int i = 0; i < 3; i++)
        this->links[i].type = RobotLink3D::Prismatic;
    for (int i = 3; i < 6; i++)
        this->links[i].type = RobotLink3D::Revolute;

    //w: axis for the link movement in link's local frame
    this->links[0].w.set(1, 0, 0);
    this->links[1].w.set(0, 1, 0);
    this->links[2].w.set(0, 0, 1);
    this->links[3].w.set(0, 0, 1);
    this->links[4].w.set(0, 1, 0);
    this->links[5].w.set(1, 0, 0);
    for (int i = 0; i < 5; i++) {
        this->links[i].com.setZero();
        this->links[i].mass = 0;
        this->links[i].inertia.setZero();
        this->links[i].T0_Parent.setIdentity();
    }

    //floating base joint
    this->joints.resize(1 + parser->joints_.size());
    this->joints[0].type = RobotJoint::Floating;
    this->joints[0].linkIndex = 5;
    this->joints[0].baseIndex = -1;

    boost::shared_ptr<urdf::Link> root_link = parser->root_link_;
    if (!root_link) {
        cout << "Root link is NULL!" << endl;
        return false;
    }

    // -------------------------------

    URDFLinkNode rootLinkNode(root_link, 0, -1);
    std::vector<URDFLinkNode> linkNodes;
    URDFConverter::DFSLinkTree(rootLinkNode, linkNodes);

    std::vector<boost::shared_ptr<urdf::Joint> > urdfJoints;
    for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it =
         parser->joints_.begin(); it != parser->joints_.end(); ++it) {
        boost::shared_ptr<urdf::Joint> joint = it->second;
        urdfJoints.push_back(joint);
    }

    URDFConverter::setJointforNodes(urdfJoints, linkNodes);
    URDFConverter::processTParentTransformations(linkNodes);

    double default_mass = 0.001;
    Matrix3 default_inertia; default_inertia.setIdentity(); default_inertia *= 0.00001;

    for (int i = 0; i < linkNodes.size(); i++)
    {
        URDFLinkNode* linkNode = &linkNodes[i];
        int link_index = linkNode->index + 5;

        this->linkNames[link_index] = linkNode->link->name; //done
        this->parents[link_index] = linkNode->index_parent + 5;

        this->links[link_index].type = RobotLink3D::Revolute; //Check correctness
        this->links[link_index].T0_Parent.setIdentity();
        if (link_index > 5)
            this->links[link_index].w.set(linkNode->axis);
        if(this->links[link_index].w.norm() < 0.1){
            cout<<linkNames[link_index]<<";"<<this->links[link_index].w<<endl;
            getchar();
        }

        // If have inertia specified, then, use the specified inertia
        if (linkNode->link->inertial) {
            this->links[link_index].com = linkNode->T_link_to_inertia.t;
            this->links[link_index].mass = linkNode->link->inertial->mass; //done
            Matrix3 ori_inertia = URDFConverter::convertInertial(
                        *linkNode->link->inertial);
            this->links[link_index].inertia.mul(
                        linkNode->T_link_to_inertia_inverse.R, ori_inertia);
        }
        // Otherwise, set it to default value
        else {
            this->links[link_index].com = Vector3(0, 0, 0);
            this->links[link_index].mass = default_mass;
            this->links[link_index].inertia = default_inertia;
        }

        // At first, set them to be default Inf value; it will be modified in later steps.
        this->accMax[link_index] = Inf;
        this->qMax[link_index] = Inf;
        this->qMin[link_index] = -Inf;
        this->velMax[link_index] = Inf;
        this->velMin[link_index] = -Inf;
        this->torqueMax[link_index] = Inf;
        this->powerMax[link_index] = Inf;
        //set joint
        urdf::Joint* joint = linkNode->joint;
        if (joint) {
            int joint_index = link_index - 5;
            this->joints[joint_index].type = URDFConverter::jointType_URDF2ROB(
                        joint->type); //done. finish function
            if (joint->type == urdf::Joint::PRISMATIC){
                this->links[link_index].type = RobotLink3D::Prismatic;
            }
            this->joints[joint_index].linkIndex = link_index; //done

            this->links[link_index].T0_Parent.set(linkNode->T_parent);

            if (joint->limits) {
                this->qMax[link_index] = joint->limits->upper;
                this->qMin[link_index] = joint->limits->lower;
                this->velMax[link_index] = joint->limits->velocity;
                this->velMin[link_index] = -joint->limits->velocity;
                this->torqueMax[link_index] = joint->limits->effort; //TODO: in URDF, no explicit value specified, effort has unit n*m,
            }
            if(this->joints[joint_index].type == RobotJoint::Weld){
                qMin[link_index] = 0;
                qMax[link_index] = 0;
            }
            if (this->joints[joint_index].type == RobotJoint::Normal
                    || this->joints[joint_index].type == RobotJoint::Spin) {
                int linkI = this->joints[joint_index].linkIndex;
                RobotJointDriver driver;
                driverNames.push_back(linkNames[linkI]);
                driver.type = RobotJointDriver::Normal;
                driver.linkIndices.push_back(linkI);
                driver.qmin = qMin(linkI);
                driver.qmax = qMax(linkI);
                driver.vmin = velMin(linkI);
                driver.vmax = velMax(linkI);
                driver.tmin = -torqueMax(linkI);
                driver.tmax = torqueMax(linkI);
                driver.amin = -accMax(linkI);
                driver.amax = accMax(linkI);
                driver.servoP = 100;
                driver.servoI = 0;
                driver.servoD = 10;
                driver.dryFriction = 0;
                if (joint->dynamics)
                    driver.dryFriction =
                            joint->dynamics->friction;

                drivers.push_back(driver);
            }
        }
    }

    this->UpdateConfig(q);
    cout << "Done loading robot file.\n" << endl;
    return true;
}
}
