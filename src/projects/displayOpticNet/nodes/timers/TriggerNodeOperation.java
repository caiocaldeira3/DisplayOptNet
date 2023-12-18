package projects.displayOpticNet.nodes.timers;

import projects.displayOpticNet.CustomGlobal;
import projects.bstOpticalNet.nodes.nodeImplementations.NetworkNode;

import sinalgo.nodes.timers.Timer;
import sinalgo.tools.Tools;

/**
 * For each TriggerNodeOperation is set a timer. When the timer runs out the it releases
 * the message from src to destination to the respective srcNode
 */
public class TriggerNodeOperation extends Timer {

    public int src;
    public int dst;

    /**
     * Constructor sets the src and destination for the message that will be created
     * when it is fired
     * @param src   the src node for the new message
     * @param dst   the dst node for the new message
     */
    public TriggerNodeOperation (int src, int dst) {
        this.src = src;
        this.dst = dst;
    }

    /**
     * When the timer runs out this method is called, saving the new message on the NetworkNode
     */
    @Override
    public void fire () {
        NetworkNode srcnode = (NetworkNode) Tools.getNodeByID(this.src);
        srcnode.newMessage(this.dst);

        CustomGlobal.mustGenerateSplay = true;
    }

}
