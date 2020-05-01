#include "ReducedSimulator.h"
#include "imgui.h"
#include "LoboDynamic/LoboDynamic.h"

#include "ReducedKineticModel.h"
#include "ReducedTimeIntegration.h"

#include "Functions/EigenMatrixIO.h"

Lobo::ReducedSimulator::ReducedSimulator(Lobo::LoboDynamicScene *parent_scene_)
    : DynamicSimulator(parent_scene_)
{
    constrainmodel = NULL;
    collisionmodel = NULL;
    hyperelastic_model = NULL;
}

Lobo::ReducedSimulator::~ReducedSimulator()
{
    delete constrainmodel;
    delete collisionmodel;
    delete hyperelastic_model;
}

void Lobo::ReducedSimulator::drawImGui()
{
    ImGui::Text("ReducedSimulator solver");
    DynamicSimulator::drawImGui();
}

void Lobo::ReducedSimulator::runXMLscript(pugi::xml_node &solver_node)
{
    DynamicSimulator::runXMLscript(solver_node);

    if (solver_node.child("ConstraintModel"))
    {
        pugi::xml_node model_node = solver_node.child("ConstraintModel");

        double weight = model_node.attribute("weight").as_double();
        constrainmodel = new Lobo::ConstrainModel(bind_tetMesh);
        constrainmodel->runXMLscript(model_node);
        models.push_back(constrainmodel);
    }

    if (solver_node.child("CollisionModel"))
    {
        pugi::xml_node model_node = solver_node.child("CollisionModel");
        collisionmodel = new Lobo::CollisionModel(bind_tetMesh);
        collisionmodel->runXMLscript(model_node);
        models.push_back(collisionmodel);
    }

    if (solver_node.child("HyperelasticModel"))
    {
        pugi::xml_node modelnode = solver_node.child("HyperelasticModel");
        hyperelastic_model =
            new Lobo::HyperelasticModel(parent_scene, bind_tetMesh);
        hyperelastic_model->runXMLscript(modelnode);
        models.push_back(hyperelastic_model);
    }

    if (solver_node.child("KineticModel"))
    {
        pugi::xml_node modelnode = solver_node.child("KineticModel");

        std::string filepath = modelnode.child("phi_path").text().as_string();
        std::string global_path = Lobo::getPath(filepath.c_str());
        Eigen::MatrixXd phi;
        std::cout<<filepath<<std::endl;
        EigenMatrixIO::read_binary(global_path.c_str(), phi);
        std::cout<<phi.rows()<<" " << phi.cols()<<std::endl;
        reduced_kinetic_model = new Lobo::ReducedKineticModel(parent_scene, &phi, bind_tetMesh, hyperelastic_model, constrainmodel, collisionmodel);
    }

    if (solver_node.child("TimeIntegration"))
    {
        pugi::xml_node modelnode = solver_node.child("TimeIntegration");
        if (strcmp(modelnode.attribute("method").as_string(),
                   "ImplicitDense") == 0)
        {
            double damping_ratio = 0.99;
            double time_step = 0.01;
            int skip_step = 1;
            int flags = 0;

            if (modelnode.attribute("damping"))
                damping_ratio = modelnode.attribute("damping").as_double();

            if (modelnode.attribute("timestep"))
                time_step = modelnode.attribute("timestep").as_double();

            if (modelnode.attribute("skipsteps"))
                skip_step = modelnode.attribute("skipsteps").as_int();

            if (modelnode.attribute("recordq"))
                if (modelnode.attribute("recordq").as_bool())
                {
                    flags |= IntegratorFlags_recordq;
                }

            int full_DOFs = bind_tetMesh->getNumVertex() * 3;
            reduced_time_integration = new Lobo::ReducedTimeIntegration(
                reduced_kinetic_model, full_DOFs, damping_ratio,
                time_step, skip_step, flags);
            reduced_time_integration->runXMLscript(modelnode);
        }
    }

    if (solver_node.attribute("precompute").as_bool())
    {
        precompute();
    }
}

void Lobo::ReducedSimulator::precompute()
{
    DynamicSimulator::precompute();

    reduced_kinetic_model->precompute();
    reduced_time_integration->precompute();
}

void Lobo::ReducedSimulator::stepForward()
{
    int step = reduced_time_integration->step;
    double scale = std::sin(step * 0.1) * 0.2;

    if (step % 400 > 200)
    {
        scale = 0.0;
    }

    reduced_kinetic_model->external_forces = reduced_kinetic_model->gravity_force * scale;

    //kinetic_model->external_forces.setZero();
    reduced_kinetic_model->external_forces += bind_tetMesh->tet_vertice_force * 5.0;

    reduced_time_integration->stepFoward();

    bind_tetMesh->updateTetVertices(&(reduced_time_integration->q));
}

int Lobo::ReducedSimulator::getCurStep()
{
    int tmp = 0;
    if (reduced_time_integration)
    {
        tmp = reduced_time_integration->step;
    }
    return tmp;
}