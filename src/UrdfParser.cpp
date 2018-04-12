#include "UrdfParser.hpp"

#include <fstream>
#include <sstream>

using namespace std;
using namespace PositionManager;

UrdfParser::UrdfParser()
{}

void UrdfParser::reset()
{
    _frameIds.clear();
    _poses.clear();
}

int UrdfParser::parseURDF(string path)
{
    this->reset();
    
    urdf::ModelInterfaceSharedPtr model;
    
    // following code extracted from urdfdom_model.cpp
    ifstream stream(path.c_str());
    if (!stream)
    {
        cout << "File " + path + " does not exist" << endl;
        return 0;
    }
    string xmlStr((istreambuf_iterator<char>(stream)), istreambuf_iterator<char>());
    model = urdf::parseURDF(xmlStr);
    if(!model)
    {
        cout << "Error UrdfLoader : unable to load " << path << endl;
        return 0;
    }
    
    // Covariances are not a standard part of urdf joints but can still be parsed in the xml file.
    // A better way would be to rewrite the urdf pose to handle covariances
    map<string,Covariance> jointCovariances;
    this->extractCovariances(xmlStr, jointCovariances);

    if(!model->getRoot())
    {
        cout << "Root not defined ?" << endl;
        return 0;
    }

    _rootFrameId = model->getRoot()->name;

    // Links and Joints are stored in a std::map
    // Links in urdf are Frames in Envire
    for(map<string,urdf::LinkSharedPtr>::const_iterator link = model->links_.begin();
        link != model->links_.end(); link++)
    {
        _frameIds.push_back(FrameId(link->second->name));
    }
   
    // Joints in urdf are Transforms in envire
    Pose pose;
    for(map<string,urdf::JointSharedPtr>::const_iterator joint = model->joints_.begin();
        joint != model->joints_.end(); joint++)
    {
        urdf::Pose& urdfTr = joint->second->parent_to_joint_origin_transform;
       
        pose._parent = joint->second->parent_link_name;
        pose._child  = joint->second->child_link_name;
        
        pose._tr.transform.translation = base::Position(urdfTr.position.x,
                                                        urdfTr.position.y,
                                                        urdfTr.position.z);
        pose._tr.transform.orientation = base::Quaterniond(urdfTr.rotation.w,
                                                           urdfTr.rotation.x,
                                                           urdfTr.rotation.y,
                                                           urdfTr.rotation.z);
        pose._tr.transform.orientation.normalize();
        try
        {
            pose._tr.transform.cov = jointCovariances.at(joint->second->name);
        }
        catch(const out_of_range& e)
        {
            cout << "Fatal Error UrdfLoader : joint \"" << e.what() << "\"  not found when assigning covariance, using default value cov = 1e-6*Identity" << endl;
            pose._tr.transform.cov = 1e-6*base::Matrix6d::Identity();
        }

        if(joint->second->type != urdf::Joint::FIXED)
        {
            _movableJoints.insert(pair<string,Joint>(joint->second->name, Joint(joint->second->parent_link_name, joint->second->child_link_name)));
            cout << "Added movable joint : " << joint->second->name << endl;
        }
        
        _poses.push_back(pose);
    }

    /*for(list<Pose>::iterator it = _poses.begin(); it != _poses.end(); ++it)
    {
        cout << it->toStringVerbose() << endl << endl;
    }*/

    return 1;
}

int UrdfParser::extractCovariances(const string& xmlStr, map<string,Covariance>& jointCovariances)
{
    TiXmlDocument xmlDoc;
    xmlDoc.Parse(xmlStr.c_str());
    TiXmlElement *robotXml = xmlDoc.FirstChildElement("robot");
	
    for(TiXmlElement* jointXml = robotXml->FirstChildElement("joint"); jointXml; jointXml = jointXml->NextSiblingElement("joint"))
	{
        Covariance cov;
        TiXmlElement* covXml = jointXml->FirstChildElement("covariance");
        if(!covXml)
        {
            cout << "Warning UrdfParser : Missing covariance element in joint definition, using default value cov = 1e-6*Identity" << endl;
            cov = 1e-6*base::Matrix6d::Identity();
            jointCovariances[string(jointXml->Attribute("name"))] = cov;
            continue;
        }

        const char* covStr = covXml->Attribute("mat");
        if(!covStr)
        {
            cout << "Warning UrdfParser : Missing mat attribute in covariance definition, using default value cov = 1e-6*Identity" << endl;
            cov = 1e-6*base::Matrix6d::Identity();
            jointCovariances[string(jointXml->Attribute("name"))] = cov;
            continue;
        }
        
        if(!this->parseCovarianceString(covStr,cov))
        {
            cout << "Error UrdfParser : Invalid covariance matrix string, using default value cov = 1e-6*Identity" << "\n" << "String is :" << "\n" << covStr << endl;
            cov = 1e-6*base::Matrix6d::Identity();
            jointCovariances[string(jointXml->Attribute("name"))] = cov;
            continue;
        }
        jointCovariances[string(jointXml->Attribute("name"))] = cov;
	}
}

int UrdfParser::parseCovarianceString(const char* strIn, Covariance& cov)
{
    double matValues[36];
    string str(strIn);
    str = str + " ";

    for(int i = 0; i < str.size(); i++)
    {
        if(str[i] >= '0' && str[i] <= '9' || str[i] == '.' || str[i] == '-' || str[i] == 'e')
            continue;
        else
            str[i] = ' ';
    }

    istringstream iss(str);
    for(int i = 0; i < 36; i++)
    {
        iss >> matValues[i];
        if(!iss.good())
            return 0;
    }

    Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> eigMap(matValues);
    cov = eigMap;

    return 1;
}


