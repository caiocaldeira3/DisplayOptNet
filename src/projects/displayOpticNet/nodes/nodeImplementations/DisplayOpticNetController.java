package projects.displayOpticNet.nodes.nodeImplementations;

import java.util.ArrayList;

import projects.bstOpticalNet.nodes.messages.HasMessage;
import projects.bstOpticalNet.nodes.models.Direction;
import projects.bstOpticalNet.nodes.models.InfraNode;
import projects.bstOpticalNet.nodes.models.Rotation;
import projects.bstOpticalNet.nodes.nodeImplementations.NetworkController;
import projects.bstOpticalNet.nodes.nodeImplementations.NetworkNode;

import sinalgo.tools.Tools;

/**
 * The CBNetController implements the remaining abstract methods left by the NetworkControllers
 * it's constructor calls it's parent class constructor. This layer manages the management of the
 * node weight and counter, and uses as an extra check before performing a rotation.
 * This class implements the blunt of the CBNet algortithm over the OpticalNet framework.
 */
public class DisplayOpticNetController extends NetworkController {
    /**
     * Initializes the CBNetController and makes a call for it's parent constructor.
     * This constructor builds the network as a balanced BST.
     * @param numNodes      Number of nodes in the network
     * @param switchSize    Number of input/output ports in the switch
     * @param netNodes      Array with the initialized NetworkNodes
     */
    public DisplayOpticNetController (
    		int numNodes, int switchSize, ArrayList<NetworkNode> netNodes, boolean mirrored
    ) {
        super(numNodes, switchSize, netNodes, mirrored);
        this.projectName = "displayOpticNet";
    }

    /**
     * Initializes the CBNetController and makes a call for it's parent constructor. If an
     * edgeList is provided the tree topology follow the specified one. If the edge list
     * can't build an BST, the constructor builds a balanced BST instead.
     * @param numNodes      Number of nodes in the network
     * @param switchSize    Number of input/output ports in the switch
     * @param netNodes      Array with the initialized NetworkNodes
     * @param edgeList      Array with the network edges, if provided.
     */
    public DisplayOpticNetController (
        int numNodes, int switchSize, ArrayList<NetworkNode> netNodes, ArrayList<Integer> edgeList, boolean mirrored
    ) {
        super(numNodes, switchSize, netNodes, edgeList, mirrored);
        this.projectName = "displayOpticNet";
    }

    @Override
    public void controllerStep () {
        super.controllerStep();
    }

            /**
     * Getter for the rotation a node should perfomed to rout a message to the destination node.
     * if the message is at one hop away to it's destination or to the LCA between the src node
     * and the dst node the message is simply routed one time. Else, it returns the appropriated
     * rotation based on the direction the message needs to be routed and the network topology
     * surrounding the involved nodes.
     * @param x         InfraNode with the message
     * @param dstNode   destination InfraNode
     * @return          the decided rotation
     */
    @Override
    protected Rotation getRotationToPerform (InfraNode x, InfraNode dstNode) {
        Direction direction = x.getRoutingDirection(dstNode);

        if (direction == Direction.PARENTROUT) {
            return Rotation.NULL;

        } else if (direction == Direction.LEFTROUT) {
            return Rotation.NULL;

        } else if (direction == Direction.RIGHTROUT) {
            return Rotation.NULL;

        } else if (
            direction == Direction.PARENT &&
            !(this.isValidNode(x.getParent()) && this.isValidNode(x.getParent().getParent()))
        ) {
            return Rotation.NULL;

        }

        /*bottom-up - BEGIN*/
        if (direction == Direction.PARENT) {
            InfraNode y = x.getParent();
            InfraNode z = y.getParent();
            Direction parentDir = y.getRoutingDirection(dstNode);

            if (parentDir != Direction.PARENT) {
                return Rotation.ZIG_BOTTOMUP;

            } else if (
                this.isValidNode(y.getLeftChild()) && x == y.getLeftChild() &&
                this.isValidNode(z.getLeftChild()) && y == z.getLeftChild()
            ) {
                return Rotation.ZIGZIGLEFT_BOTTOMUP;

            } else if (
                this.isValidNode(y.getRightChild()) && x == y.getRightChild() &&
                this.isValidNode(z.getRightChild()) && y == z.getRightChild()
            ) {
                return Rotation.ZIGZIGRIGHT_BOTTOMUP;

            } else if (
                this.isValidNode(y.getRightChild()) && x == y.getRightChild() &&
                this.isValidNode(z.getLeftChild()) && y == z.getLeftChild()
            ) {
                return Rotation.ZIGZAGLEFT_BOTTOMUP;

            } else if (
                this.isValidNode(y.getLeftChild()) && x == y.getLeftChild() &&
                this.isValidNode(z.getRightChild()) && y == z.getRightChild()
            ) {
                return Rotation.ZIGZAGRIGHT_BOTTOMUP;

            } else {
                Tools.fatalError("Network topology for BottomUp not expected");

            }
        }

        return Rotation.NULL;
    }

    /**
     * This method locks the routing nodes and then, for every node with a message, if it is
     * possible, performs the rotation specified by the getRotationToPerformed, and if it is not
     * tries to rout the message 2 times. If neither step is possible, due to involved nodes being
     * locked or other issues, the node is not allowed to act in this round.
     */
    @Override
    protected void updateConn () {
        this.lockRoutingNodes();

        while (!this.nodesWithMsg.isEmpty()) {
            HasMessage hasmsg = this.nodesWithMsg.poll();
            int nodeId = hasmsg.getCurrId();

            InfraNode node = this.getInfraNode(nodeId);
            InfraNode dstNode = this.getInfraNode(hasmsg.getDst());

            Rotation cOp = this.getRotationToPerform(node, dstNode);
            Rotation dOp = this.getRotationToPerform(dstNode, node);

            if (cOp == Rotation.NULL && dOp == Rotation.NULL) {
                if (this.allowRouting(node, dstNode, 1)) {
                    System.out.println("Rout message to destination");

                }

                continue;
            }

            boolean active = false;
            switch (cOp) {
                case NULL:
                    break;

                case ZIG_BOTTOMUP:
                	if (this.zigBottomUp(node)) {
                		System.out.println("zigBottomUp");
                        active = true;

                	}
                    break;

                case ZIGZIGLEFT_BOTTOMUP:
                case ZIGZIGRIGHT_BOTTOMUP:
                    if (this.zigZigBottomUp(node)) {
                        System.out.println("zigZigBottomUp");
                        active = true;

                    }
                    break;

                case ZIGZAGLEFT_BOTTOMUP:
                case ZIGZAGRIGHT_BOTTOMUP:
                    if (this.zigZagBottomUp(node)) {
                        System.out.println("zigZagBottomUp");
                        active = true;

                    }
                    break;

                default:
                    break;
            }

            switch (dOp) {
                case NULL:
                    break;

                case ZIG_BOTTOMUP:
                	if (this.zigBottomUp(dstNode)) {
                		System.out.println("zigBottomUp");
                        active = true;

                	}
                    break;

                case ZIGZIGLEFT_BOTTOMUP:
                case ZIGZIGRIGHT_BOTTOMUP:
                    if (this.zigZigBottomUp(dstNode)) {
                        System.out.println("zigZigBottomUp");
                        active = true;

                    }
                    break;

                case ZIGZAGLEFT_BOTTOMUP:
                case ZIGZAGRIGHT_BOTTOMUP:
                    if (this.zigZagBottomUp(dstNode)) {
                        System.out.println("zigZagBottomUp");
                        active = true;

                    }
                    break;

                default:
                    break;
            }

            this.areAvailableNodes(node);
            this.areAvailableNodes(dstNode);

            if (active) {
                this.logIncrementActiveRequests();

            }
        }
    }
}
