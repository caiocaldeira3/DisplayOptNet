package projects.displayOpticNet;

import java.util.ArrayList;
import java.util.Random;

import projects.displayOpticNet.nodes.timers.TriggerNodeOperation;
import projects.displayOpticNet.nodes.nodeImplementations.DisplayOpticNetController;
import projects.bstOpticalNet.nodes.nodeImplementations.NetworkNode;
import projects.defaultProject.RequestQueue;

import sinalgo.configuration.Configuration;
import sinalgo.runtime.AbstractCustomGlobal;
import sinalgo.tools.Tools;
import sinalgo.tools.Tuple;

public class CustomGlobal extends AbstractCustomGlobal {

    /* Final Condition */
    public long MAX_REQ;

    /* Simulation */
    public DisplayOpticNetController controller = null;
    public ArrayList<NetworkNode> netNodes = new ArrayList<>();

    public RequestQueue requestQueue;

    /* Control Execution */
    public boolean isSequential = true;
    public static boolean mustGenerateSplay = true;

    public Random random = Tools.getRandomNumberGenerator();
    public double lambda = 0.05;

    @Override
    public void preRun () {
        String input = "";
        String output = "";
        Integer switchSize = -1;
        Integer numberOfNodes = -1;
        boolean mirrored = false;

        try {

            if (Configuration.hasParameter("input")) {
                input = Configuration.getStringParameter("input");
            }

            this.requestQueue = new RequestQueue(input);
            numberOfNodes = this.requestQueue.getNumberOfNodes();
            MAX_REQ = this.requestQueue.getNumberOfRequests();

            if (Configuration.hasParameter("output")) {
                output = Configuration.getStringParameter("output");
            }

            if (Configuration.hasParameter("mu")) {
                double mu = (double) Configuration.getIntegerParameter("mu");
                lambda = (double) (1 / mu);
            }

            if (Configuration.hasParameter("switchSize")) {
                switchSize = Configuration.getIntegerParameter("switchSize");

            } else {
                switchSize = 2 * numberOfNodes;

            }

            if (Configuration.hasParameter("mirrored")) {
                mirrored = Configuration.getBooleanParameter("mirrored");

            } else {
                mirrored = true;
            }

            if (Configuration.hasParameter("isSequential")) {
                this.isSequential = Configuration.getBooleanParameter("isSequential");
            }

        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Missing configuration parameters");
        }

        for (int i = 0; i <= numberOfNodes; i++) {
            NetworkNode newNetNode = new NetworkNode();
            newNetNode.finishInitializationWithDefaultModels(true);
            netNodes.add(newNetNode);
        }

        this.controller = new DisplayOpticNetController(
            numberOfNodes, switchSize, netNodes, mirrored
        );
        this.controller.finishInitializationWithDefaultModels(true);

        /* Set Log Path */
        this.controller.setLogPath(output);

        for (int i = 0; i < numberOfNodes; i++) {
            netNodes.get(i).setController(this.controller);
        }
    }

    @Override
    public void preRound () {
        if (this.isSequential && this.controller.getSeq()) {
            Tuple<Integer, Integer> r = this.requestQueue.getNextRequest();
            NetworkNode srcnode = (NetworkNode) Tools.getNodeByID(r.first + 1);
            srcnode.newMessage(r.second + 1);

            this.controller.setSeq();

        } else if (!this.isSequential && mustGenerateSplay && this.requestQueue.hasNextRequest()) {
                mustGenerateSplay = false;

                double u = random.nextDouble();
                double x = Math.log(1 - u) / (-lambda);
                x = (int) x;
                if (x <= 0) {
                    x = 1;
                }

                Tuple<Integer, Integer> r = this.requestQueue.getNextRequest();
                TriggerNodeOperation ted = new TriggerNodeOperation(r.first + 1, r.second + 1);
                ted.startGlobalTimer(x);
        }

    }

    @Override
    public boolean hasTerminated () {
        if (this.controller.getCompletedRequests() == MAX_REQ) {
            this.controller.logEndOfSimulation();

            return true;

        }

        return false;
    }

}
