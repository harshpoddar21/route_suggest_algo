package com.graphhopper.jsprit.core.algorithm;

import com.graphhopper.jsprit.core.algorithm.state.InternalStates;
import com.graphhopper.jsprit.core.problem.solution.SolutionCostCalculator;
import com.graphhopper.jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import com.graphhopper.jsprit.core.problem.solution.route.VehicleRoute;
import com.graphhopper.jsprit.core.problem.solution.route.activity.TourActivity;
import com.graphhopper.jsprit.core.problem.solution.route.state.RouteAndActivityStateGetter;
import com.graphhopper.jsprit.core.problem.vehicle.Vehicle;
import com.graphhopper.jsprit.core.util.DistanceUnit;
import com.graphhopper.jsprit.core.util.GreatCircleCosts;

/**
 * Created by root on 20/5/16.
 */
public class RouteSuggestionCustomCostCalculator {


    private static final double REVENUE_PER_KM = 5;
    private RouteAndActivityStateGetter stateManager;

    public RouteSuggestionCustomCostCalculator(RouteAndActivityStateGetter stateManager) {
        super();
        this.stateManager = stateManager;
    }

    public SolutionCostCalculator createCalculator() {
        return new SolutionCostCalculator() {

            @Override
            public double getCosts(VehicleRoutingProblemSolution solution) {
                double c = 0.0;
                for (VehicleRoute r : solution.getRoutes()) {
                    c += stateManager.getRouteState(r, InternalStates.COSTS, Double.class);
                    c += getFixedCosts(r.getVehicle());
                    c-=getRevenue(r);
                }

                //c += solution.getUnassignedJobs().size() * c * .1;
                return c;
            }

            private double getRevenue(VehicleRoute vehicleRoute){

                double totalRevenue=0;
                GreatCircleCosts greatCircleCosts=new GreatCircleCosts(DistanceUnit.Meter,5,1.6);
                for (TourActivity tourActivity:vehicleRoute.getActivities()){


                    totalRevenue+=REVENUE_PER_KM*(greatCircleCosts.getDistance(tourActivity.getLocation(),vehicleRoute.getEnd().getLocation())/1000)*tourActivity.getSize().get(0);

                }
                return totalRevenue;
            }

            private double getFixedCosts(Vehicle vehicle) {
                if (vehicle == null) return 0.0;
                if (vehicle.getType() == null) return 0.0;
                return vehicle.getType().getVehicleCostParams().fix;
            }
        };
    }

}
