package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PolynomialRegression {

    public static class FitResult {
        public final double a;
        public final double b;
        public final double c;
        public final double rSquared;
        public final boolean isValid;

        public FitResult(double a, double b, double c, double rSquared, boolean isValid) {
            this.a = a;
            this.b = b;
            this.c = c;
            this.rSquared = rSquared;
            this.isValid = isValid;
        }

        @Override
        public String toString() {
            if (!isValid) return "INVALID (sub 3 distanțe distincte cu separare min 8 in)";
            return String.format(java.util.Locale.US, "y = %.8f x² + %.6f x + %.4f (R²=%.4f)", a, b, c, rSquared);
        }
    }

    /**
     * Verifică dacă există cel puțin `minClusters` distanțe distincte separate de minim `minSeparationInches`.
     */
    public static boolean hasMinimumDistinctClusters(List<Double> xList, int minClusters, double minSeparationInches) {
        if (xList == null || xList.size() < minClusters) return false;

        List<Double> sorted = new ArrayList<>(xList);
        Collections.sort(sorted);

        int clusters = 1;
        double lastClusterX = sorted.get(0);

        for (int i = 1; i < sorted.size(); i++) {
            double currentX = sorted.get(i);
            if ((currentX - lastClusterX) >= minSeparationInches) {
                clusters++;
                lastClusterX = currentX;
            }
        }

        return clusters >= minClusters;
    }

    public static FitResult fitQuadratic(List<Double> xList, List<Double> yList) {
        if (xList == null || yList == null || xList.size() < 3 || xList.size() != yList.size()) {
            return new FitResult(0.0, 0.0, 0.0, 0.0, false);
        }

        // Verificare existență 3 grupuri de distanțe distincte (separație de minim 8.0 inchi între ele)
        if (!hasMinimumDistinctClusters(xList, 3, 8.0)) {
            return new FitResult(0.0, 0.0, 0.0, 0.0, false);
        }

        int n = xList.size();

        double sumX = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0;
        double sumY = 0, sumXY = 0, sumX2Y = 0;

        for (int i = 0; i < n; i++) {
            double x = xList.get(i);
            double y = yList.get(i);

            double x2 = x * x;
            double x3 = x2 * x;
            double x4 = x3 * x;

            sumX += x;
            sumX2 += x2;
            sumX3 += x3;
            sumX4 += x4;

            sumY += y;
            sumXY += x * y;
            sumX2Y += x2 * y;
        }

        double m00 = sumX4, m01 = sumX3, m02 = sumX2;
        double m10 = sumX3, m11 = sumX2, m12 = sumX;
        double m20 = sumX2, m21 = sumX,  m22 = n;

        double det = m00 * (m11 * m22 - m12 * m21)
                   - m01 * (m10 * m22 - m12 * m20)
                   + m02 * (m10 * m21 - m11 * m20);

        if (Math.abs(det) < 1e-12) {
            // Regresie liniară de rezervă (a = 0)
            double denom = n * sumX2 - sumX * sumX;
            if (Math.abs(denom) < 1e-12) {
                return new FitResult(0.0, 0.0, sumY / n, 0.0, false);
            }
            double slope = (n * sumXY - sumX * sumY) / denom;
            double intercept = (sumY - slope * sumX) / n;

            double meanY = sumY / n;
            double sst = 0, sse = 0;
            for (int i = 0; i < n; i++) {
                double x = xList.get(i);
                double y = yList.get(i);
                double pred = slope * x + intercept;
                sst += Math.pow(y - meanY, 2);
                sse += Math.pow(y - pred, 2);
            }
            double rSquared = (sst > 1e-9) ? Math.max(0.0, 1.0 - (sse / sst)) : 1.0;
            return new FitResult(0.0, slope, intercept, rSquared, true);
        }

        // Regula lui Cramer pentru parabola pătratică
        double v0 = sumX2Y, v1 = sumXY, v2 = sumY;

        double detA = v0  * (m11 * m22 - m12 * m21)
                    - m01 * (v1  * m22 - m12 * v2)
                    + m02 * (v1  * m21 - m11 * v2);

        double detB = m00 * (v1  * m22 - m12 * v2)
                    - v0  * (m10 * m22 - m12 * m20)
                    + m02 * (m10 * v2  - v1  * m20);

        double detC = m00 * (m11 * v2  - v1  * m21)
                    - m01 * (m10 * v2  - v1  * m20)
                    + v0  * (m10 * m21 - m11 * m20);

        double a = detA / det;
        double b = detB / det;
        double c = detC / det;

        // Calcul R^2
        double meanY = sumY / n;
        double sst = 0, sse = 0;

        for (int i = 0; i < n; i++) {
            double x = xList.get(i);
            double y = yList.get(i);
            double pred = a * x * x + b * x + c;

            sst += Math.pow(y - meanY, 2);
            sse += Math.pow(y - pred, 2);
        }

        double rSquared = (sst > 1e-9) ? Math.max(0.0, 1.0 - (sse / sst)) : 1.0;

        return new FitResult(a, b, c, rSquared, true);
    }
}
