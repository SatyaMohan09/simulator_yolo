package com.evtol.trajectoryengine.service;

import org.springframework.stereotype.Service;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.util.Base64;

@Service
public class LocalVisionProcessingService {

    public String edgeDetection(String frameDataUrl) throws Exception {
        BufferedImage input = decodeFrame(frameDataUrl);
        int rows = input.getHeight();
        int cols = input.getWidth();

        double[][] rgb = imageToRgbMatrices(input);
        double[] edges = applyEdgeDetection(rgb[0], rgb[1], rgb[2], rows, cols);
        BufferedImage output = rgbMatricesToImage(edges, edges, edges, rows, cols);
        return encodePngDataUrl(output);
    }

    public String thresholdSegmentation(String frameDataUrl) throws Exception {
        BufferedImage input = decodeFrame(frameDataUrl);
        int rows = input.getHeight();
        int cols = input.getWidth();

        double[][] rgb = imageToRgbMatrices(input);
        double[] segmented = applyThreshold(rgb[0], rgb[1], rgb[2], rows, cols);
        BufferedImage output = rgbMatricesToImage(segmented, segmented, segmented, rows, cols);
        return encodePngDataUrl(output);
    }

    private BufferedImage decodeFrame(String frameDataUrl) throws Exception {
        if (frameDataUrl == null || frameDataUrl.isBlank()) {
            throw new IllegalArgumentException("Missing frame");
        }

        String base64 = frameDataUrl.contains(",")
                ? frameDataUrl.substring(frameDataUrl.indexOf(',') + 1)
                : frameDataUrl;

        byte[] imageBytes = Base64.getDecoder().decode(base64);
        BufferedImage image = ImageIO.read(new ByteArrayInputStream(imageBytes));
        if (image == null) {
            throw new IllegalArgumentException("Invalid image data");
        }
        return image;
    }

    private String encodePngDataUrl(BufferedImage image) throws Exception {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        ImageIO.write(image, "png", baos);
        return "data:image/png;base64," + Base64.getEncoder().encodeToString(baos.toByteArray());
    }

    private double[][] imageToRgbMatrices(BufferedImage image) {
        int rows = image.getHeight();
        int cols = image.getWidth();
        double[] red = new double[rows * cols];
        double[] green = new double[rows * cols];
        double[] blue = new double[rows * cols];

        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                int pixel = image.getRGB(col, row);
                int index = row * cols + col;
                red[index] = (pixel >> 16) & 0xFF;
                green[index] = (pixel >> 8) & 0xFF;
                blue[index] = pixel & 0xFF;
            }
        }

        return new double[][]{red, green, blue};
    }

    private double[] applyGrayscale(double[] red, double[] green, double[] blue) {
        double[] gray = new double[red.length];
        for (int i = 0; i < red.length; i++) {
            gray[i] = 0.299 * red[i] + 0.587 * green[i] + 0.114 * blue[i];
        }
        return gray;
    }

    private double[] applyEdgeDetection(double[] red, double[] green, double[] blue, int rows, int cols) {
        double[] gray = applyGrayscale(red, green, blue);
        double[] edges = new double[rows * cols];

        for (int row = 1; row < rows - 1; row++) {
            for (int col = 1; col < cols - 1; col++) {
                int index = row * cols + col;

                double gx = -gray[(row - 1) * cols + (col - 1)] - 2 * gray[row * cols + (col - 1)] - gray[(row + 1) * cols + (col - 1)]
                        + gray[(row - 1) * cols + (col + 1)] + 2 * gray[row * cols + (col + 1)] + gray[(row + 1) * cols + (col + 1)];

                double gy = -gray[(row - 1) * cols + (col - 1)] - 2 * gray[(row - 1) * cols + col] - gray[(row - 1) * cols + (col + 1)]
                        + gray[(row + 1) * cols + (col - 1)] + 2 * gray[(row + 1) * cols + col] + gray[(row + 1) * cols + (col + 1)];

                edges[index] = clamp(Math.sqrt(gx * gx + gy * gy));
            }
        }

        for (int col = 0; col < cols; col++) {
            edges[col] = 0;
            edges[(rows - 1) * cols + col] = 0;
        }
        for (int row = 0; row < rows; row++) {
            edges[row * cols] = 0;
            edges[row * cols + (cols - 1)] = 0;
        }

        return edges;
    }

    private double[] applyThreshold(double[] red, double[] green, double[] blue, int rows, int cols) {
        double[] gray = applyGrayscale(red, green, blue);
        double[] segmented = new double[rows * cols];
        int threshold = findOtsuThreshold(gray);

        for (int i = 0; i < segmented.length; i++) {
            segmented[i] = gray[i] >= threshold ? 255 : 0;
        }

        return segmented;
    }

    private int findOtsuThreshold(double[] gray) {
        int[] histogram = new int[256];
        for (double pixel : gray) {
            histogram[clamp((int) Math.round(pixel))]++;
        }

        int total = gray.length;
        double sum = 0;
        for (int i = 0; i < histogram.length; i++) {
            sum += (double) i * histogram[i];
        }

        double sumBackground = 0;
        int weightBackground = 0;
        double maxVariance = -1;
        int threshold = 0;

        for (int i = 0; i < histogram.length; i++) {
            weightBackground += histogram[i];
            if (weightBackground == 0) {
                continue;
            }

            int weightForeground = total - weightBackground;
            if (weightForeground == 0) {
                break;
            }

            sumBackground += (double) i * histogram[i];
            double meanBackground = sumBackground / weightBackground;
            double meanForeground = (sum - sumBackground) / weightForeground;
            double betweenClassVariance = (double) weightBackground * weightForeground
                    * Math.pow(meanBackground - meanForeground, 2);

            if (betweenClassVariance > maxVariance) {
                maxVariance = betweenClassVariance;
                threshold = i;
            }
        }

        return threshold;
    }

    private BufferedImage rgbMatricesToImage(double[] red, double[] green, double[] blue, int rows, int cols) {
        BufferedImage image = new BufferedImage(cols, rows, BufferedImage.TYPE_INT_RGB);
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                int index = row * cols + col;
                int rgb = (clamp((int) red[index]) << 16)
                        | (clamp((int) green[index]) << 8)
                        | clamp((int) blue[index]);
                image.setRGB(col, row, rgb);
            }
        }
        return image;
    }

    private int clamp(int value) {
        return Math.max(0, Math.min(255, value));
    }

    private double clamp(double value) {
        return Math.max(0, Math.min(255, value));
    }
}
