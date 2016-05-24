/*******************************************************************************
 * Copyright (C) 2014  Stefan Schroeder
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/
package com.graphhopper.jsprit.examples;

import com.graphhopper.jsprit.analysis.toolbox.AlgorithmEventsRecorder;
import com.graphhopper.jsprit.analysis.toolbox.AlgorithmSearchProgressChartListener;
import com.graphhopper.jsprit.analysis.toolbox.GraphStreamViewer;
import com.graphhopper.jsprit.analysis.toolbox.GraphStreamViewer.Label;
import com.graphhopper.jsprit.analysis.toolbox.Plotter;
import com.graphhopper.jsprit.core.algorithm.RouteSuggestionCustomCostCalculator;
import com.graphhopper.jsprit.core.algorithm.VehicleRoutingAlgorithm;
import com.graphhopper.jsprit.core.algorithm.VehicleRoutingAlgorithmBuilder;
import com.graphhopper.jsprit.core.algorithm.io.VehicleRoutingAlgorithms;
import com.graphhopper.jsprit.core.algorithm.selector.SelectBest;
import com.graphhopper.jsprit.core.algorithm.state.StateManager;
import com.graphhopper.jsprit.core.problem.Location;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem;
import com.graphhopper.jsprit.core.problem.constraint.ConstraintManager;
import com.graphhopper.jsprit.core.problem.constraint.HardActivityConstraint;
import com.graphhopper.jsprit.core.problem.constraint.SwitchNotFeasible;
import com.graphhopper.jsprit.core.problem.io.VrpXMLReader;
import com.graphhopper.jsprit.core.problem.job.Job;
import com.graphhopper.jsprit.core.problem.job.Pickup;
import com.graphhopper.jsprit.core.problem.misc.JobInsertionContext;
import com.graphhopper.jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import com.graphhopper.jsprit.core.problem.solution.route.activity.End;
import com.graphhopper.jsprit.core.problem.solution.route.activity.PickupService;
import com.graphhopper.jsprit.core.problem.solution.route.activity.Start;
import com.graphhopper.jsprit.core.problem.solution.route.activity.TourActivity;
import com.graphhopper.jsprit.core.reporting.SolutionPrinter;
import com.graphhopper.jsprit.core.util.DistanceUnit;
import com.graphhopper.jsprit.core.util.GreatCircleCosts;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;


public class RouteSuggestion {

    public static void main(String[] args) {
        /*
         * some preparation - create output folder
		 */
        File dir = new File("/var/www/java/jsprit/jsprit-examples/output");
        // if the directory does not exist, create it
        if (!dir.exists()) {
            System.out.println("creating directory /var/www/java/jsprit/jsprit-examples/output");
            boolean result = dir.mkdir();
            if (result) System.out.println("/output created");
        }

		/*
         * Build the problem.
		 *
		 * But define a problem-builder first.
		 */
        VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();



        vrpBuilder.setRoutingCost(new GreatCircleCosts(DistanceUnit.Meter,5,1.6));
		/*
         * A solomonReader reads solomon-instance files, and stores the required information in the builder.
		 */
        new VrpXMLReader(vrpBuilder).read("/var/www/java/jsprit/jsprit-examples/input/test_v2.xml");


		/*
         * Finally, the problem can be built. By default, transportCosts are crowFlyDistances (as usually used for vrp-instances).
		 */
        VehicleRoutingProblem vrp = vrpBuilder.build();

        Plotter pblmPlotter = new Plotter(vrp);
        pblmPlotter.plot("/var/www/java/jsprit/jsprit-examples/output/solomon_C101_specifiedVehicleEndLocations.png", "C101");

		/*
         * Define the required vehicle-routing algorithms to solve the above problem.
		 *
		 * The algorithm can be defined and configured in an xml-file.
		 */
//		VehicleRoutingAlgorithm vra = new SchrimpfFactory().createAlgorithm(vrp);
        HardActivityConstraint hardActivityConstraint=new HardActivityConstraint() {
            @Override
            public ConstraintsStatus fulfilled(JobInsertionContext iFacts, TourActivity prevAct, TourActivity newAct, TourActivity nextAct, double prevActDepTime) {


                StringBuilder stringBuilder=new StringBuilder();
                stringBuilder.append("trying route ");

                for (Job job:iFacts.getRoute().getTourActivities().getJobs()){

                    stringBuilder.append("--"+job.getId()+"--");
                }
                if (prevAct instanceof PickupService) {
                    stringBuilder.append("inserting " + iFacts.getJob().getId() + " between " + ((PickupService) prevAct).getJob().getId());
                }
                if (prevAct instanceof Start){

                    stringBuilder.append("inserting " + iFacts.getJob().getId() + " between "+((Start)prevAct).getLocation().getId());
                }
                if (nextAct instanceof PickupService){

                    stringBuilder.append(" and "+((PickupService) nextAct).getJob().getId());
                }
                if (nextAct instanceof End){

                    stringBuilder.append(" and "+((End)nextAct).getLocation().getId());
                }
                   // System.out.println(stringBuilder.toString());


                List<TourActivity> activityList=iFacts.getRoute().getActivities();
                double totalTimeTaken=0;

                double detourTime=findDetourTime(prevAct,newAct,nextAct);

                for (int i=0;i<activityList.size();i++){


                    double actualTimeTaken=findActualTimeTakenForActivity(activityList.get(i),activityList,iFacts.getRoute().getEnd().getLocation());

                    double directTimeTaken=findTimeTakenToComplete(activityList.get(i),iFacts.getRoute().getEnd().getLocation());
                    if (actualTimeTaken+detourTime-directTimeTaken>900){

                        return ConstraintsStatus.NOT_FULFILLED_BREAK;
                    }
                    if (activityList.get(i)==prevAct){

                        break;
                    }




                }


                return ConstraintsStatus.FULFILLED;
            }

            private double findActualTimeTakenForActivity(TourActivity activity, List<TourActivity> activityList, Location endLocation) {

                boolean isAfterAc=false;

                double timeTaken=0;

                for (int i=0;i<activityList.size();i++){

                    if (activityList.get(i)==activity){

                        isAfterAc=true;
                        continue;
                    }else if (isAfterAc){

                        timeTaken+=findTimeTakenBetween(activityList.get(i).getLocation(),activityList.get(i-1).getLocation());

                    }

                }
                timeTaken+=findTimeTakenToComplete(activityList.get(activityList.size()-1),endLocation);
                return timeTaken;
            }

            public double findTimeTakenToComplete(TourActivity activity,Location endLocation){
                GreatCircleCosts greatCircleCosts=new GreatCircleCosts(DistanceUnit.Meter,5,1.6);
                double timeDirect=greatCircleCosts.getTransportTime(activity.getLocation(),endLocation,0,null,null);
                return timeDirect;

            }

            public double findTimeTakenBetween(Location startLocation,Location endLocation){
                GreatCircleCosts greatCircleCosts=new GreatCircleCosts(DistanceUnit.Meter,5,1.6);
                double timeDirect=greatCircleCosts.getTransportTime(startLocation,endLocation,0,null,null);
                return timeDirect;

            }


            public double findDetourTime(TourActivity prevAct, TourActivity newAct, TourActivity nextAct){

                GreatCircleCosts greatCircleCosts=new GreatCircleCosts(DistanceUnit.Meter,5,1.6);
                double timeDirect=greatCircleCosts.getTransportTime(prevAct.getLocation(),nextAct.getLocation(),0,null,null);
                double timeVia=greatCircleCosts.getTransportTime(prevAct.getLocation(),newAct.getLocation(),0,null,null)+greatCircleCosts.getTransportTime(newAct.getLocation(),nextAct.getLocation(),0,null,null);
                return timeVia-timeDirect;


            }
        };
        HardActivityConstraint hardActivityConstraint2=new HardActivityConstraint() {
            @Override
            public ConstraintsStatus fulfilled(JobInsertionContext iFacts, TourActivity prevAct, TourActivity newAct, TourActivity nextAct, double prevActDepTime) {

                double detourTime=findDetourTime(prevAct,newAct,nextAct);

                if (detourTime>1800){

                    return ConstraintsStatus.NOT_FULFILLED;

                }
                return ConstraintsStatus.FULFILLED;
            }

            public double findDetourTime(TourActivity prevAct, TourActivity newAct, TourActivity nextAct){

                GreatCircleCosts greatCircleCosts=new GreatCircleCosts(DistanceUnit.Meter,5,1.6);
                double timeDirect=greatCircleCosts.getTransportTime(prevAct.getLocation(),nextAct.getLocation(),0,null,null);
                double timeVia=greatCircleCosts.getTransportTime(prevAct.getLocation(),newAct.getLocation(),0,null,null)+greatCircleCosts.getTransportTime(newAct.getLocation(),nextAct.getLocation(),0,null,null);
                return timeVia-timeDirect;


            }
        };
        VehicleRoutingAlgorithmBuilder vraBuilder = new VehicleRoutingAlgorithmBuilder(vrp, "/var/www/java/jsprit/jsprit-examples/input/algorithmConfigRouteSuggestion.xml");
        vraBuilder.addCoreConstraints();
        vraBuilder.setNuOfThreads(8);
        vraBuilder.addDefaultCostCalculators();
        StateManager stateManager = new StateManager(vrp);
        vraBuilder.setObjectiveFunction(new RouteSuggestionCustomCostCalculator(stateManager).createCalculator());

        ConstraintManager constraintManager = new ConstraintManager(vrp, stateManager);
        constraintManager.addConstraint(hardActivityConstraint, ConstraintManager.Priority.CRITICAL);
        vraBuilder.setStateAndConstraintManager(stateManager, constraintManager);

        VehicleRoutingAlgorithm vra = vraBuilder.build();
        //VehicleRoutingAlgorithm vra = VehicleRoutingAlgorithms.readAndCreateAlgorithm(vrp,8, "/var/www/java/jsprit/jsprit-examples/input/algorithmConfigRouteSuggestion.xml");





        vra.setMaxIterations(24000);

        vra.getAlgorithmListeners().addListener(new AlgorithmEventsRecorder(vrp,"/var/www/java/jsprit/jsprit-examples/output/sol_iter.txt"));
//		vra.setPrematureBreak(100);
        vra.getAlgorithmListeners().addListener(new AlgorithmSearchProgressChartListener("/var/www/java/jsprit/jsprit-examples/output/sol_progress.png"));
        /*
         * Solve the problem.
		 *
		 *
		 */
        Collection<VehicleRoutingProblemSolution> solutions = vra.searchSolutions();

		/*
         * Retrieve best solution.
		 */
        VehicleRoutingProblemSolution solution = new SelectBest().selectSolution(solutions);

		/*
		 * print solution
		 */
        try {
            SolutionPrinter.print(new PrintWriter("/var/www/java/jsprit/solution.txt"),vrp,solution, SolutionPrinter.Print.VERBOSE);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

		/*
		 * Plot solution.
		 */
//		SolutionPlotter.plotSolutionAsPNG(vrp, solution, "output/solomon_C101_specifiedVehicleEndLocations_solution.png","C101");
        Plotter solPlotter = new Plotter(vrp, solution);
        solPlotter.plot("/var/www/java/jsprit/jsprit-examples/output/solomon_C101_specifiedVehicleEndLocations_solution.png", "C101");


        new GraphStreamViewer(vrp, solution).setRenderDelay(50).labelWith(Label.ID).display();


    }

}
