/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package marg.images;

import java.awt.image.BufferedImage;
import java.net.URL;
import java.util.logging.Logger;
import javax.imageio.ImageIO;
import javax.swing.ImageIcon;

/**
 *
 * @author Daniel
 */
public class ResUtils {

    private static final String PATH = "/" + (ResUtils.class.getPackage().getName().replace(".", "/")) + "/";

    public static URL getURL(String name) {
        return ResUtils.class.getResource(PATH + name);
    }

    public static ImageIcon getImageIcon(String name) {
        return new ImageIcon(getBufferedImage(name));
    }

    /**
     * Finds, reads and returns an image that can be accessed by class code in a way that is independent of the location of the code.
     * The name of a resource is a '/'-separated path name that identifies the resource. Example: "marg/images/Wheel.gif"
     * @param name The / separated name of the ressource to locate the image at
     * @return the BufferedImage constructed from the given ressource. Returns a 20x20 black placeholder if no ressource was found.
     */
    public static BufferedImage getBufferedImage(String name) {
        try {
            URL url = getURL(name);
            marg.util.Log.GlobalLogger.info("Loaded image from "+ url);
            return ImageIO.read(url);
        } catch (Exception e) {
            e.printStackTrace();
            return new BufferedImage(20, 20, BufferedImage.TYPE_INT_RGB);
        }
    }

    public static void main(String[] args) {
        ResUtils.getImageIcon("GreenButton20px.gif");
    }
}