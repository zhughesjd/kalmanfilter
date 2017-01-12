package net.joshuahughes.kalmanfilter.associator;

import java.util.Arrays;
import java.util.Random;
import java.util.stream.IntStream;

public class HungarianAlgorithm {
  private final double[][] costMatrix;
  private final int rows, cols, dim;
  private final double[] labelByWorker, labelByJob;
  private final int[] minSlackWorkerByJob;
  private final double[] minSlackValueByJob;
  private final int[] matchJobByWorker, matchWorkerByJob;
  private final int[] parentWorkerByCommittedJob;
  private final boolean[] committedWorkers;

  /**
   * Construct an instance of the algorithm.
   * 
   * @param costMatrix
   *          the cost matrix, where matrix[i][j] holds the cost of assigning
   *          worker i to job j, for all i, j. The cost matrix must not be
   *          irregular in the sense that all rows must be the same length.
   */
  public HungarianAlgorithm(double[][] costMatrix) {
    this.dim = Math.max(costMatrix.length, costMatrix[0].length);
    this.rows = costMatrix.length;
    this.cols = costMatrix[0].length;
    this.costMatrix = new double[this.dim][this.dim];
    for (int w = 0; w < this.dim; w++) {
      if (w < costMatrix.length) {
        if (costMatrix[w].length != this.cols) {
          throw new IllegalArgumentException("Irregular cost matrix");
        }
        this.costMatrix[w] = Arrays.copyOf(costMatrix[w], this.dim);
      } else {
        this.costMatrix[w] = new double[this.dim];
      }
    }
    labelByWorker = new double[this.dim];
    labelByJob = new double[this.dim];
    minSlackWorkerByJob = new int[this.dim];
    minSlackValueByJob = new double[this.dim];
    committedWorkers = new boolean[this.dim];
    parentWorkerByCommittedJob = new int[this.dim];
    matchJobByWorker = new int[this.dim];
    Arrays.fill(matchJobByWorker, -1);
    matchWorkerByJob = new int[this.dim];
    Arrays.fill(matchWorkerByJob, -1);
  }

  /**
   * Compute an initial feasible solution by assigning zero labels to the
   * workers and by assigning to each job a label equal to the minimum cost
   * among its incident edges.
   */
  protected void computeInitialFeasibleSolution() {
    for (int j = 0; j < dim; j++) {
      labelByJob[j] = Double.POSITIVE_INFINITY;
    }
    for (int w = 0; w < dim; w++) {
      for (int j = 0; j < dim; j++) {
        if (costMatrix[w][j] < labelByJob[j]) {
          labelByJob[j] = costMatrix[w][j];
        }
      }
    }
  }

  /**
   * Execute the algorithm.
   * 
   * @return the minimum cost matching of workers to jobs based upon the
   *         provided cost matrix. A matching value of -1 indicates that the
   *         corresponding worker is unassigned.
   */
  public int[] execute() {
    /*
     * Heuristics to improve performance: Reduce rows and columns by their
     * smallest element, compute an initial non-zero dual feasible solution and
     * create a greedy matching from workers to jobs of the cost matrix.
     */
    reduce();
    computeInitialFeasibleSolution();
    greedyMatch();

    int w = fetchUnmatchedWorker();
    while (w < dim) {
      initializePhase(w);
      executePhase();
      w = fetchUnmatchedWorker();
    }
    int[] result = Arrays.copyOf(matchJobByWorker, rows);
    for (w = 0; w < result.length; w++) {
      if (result[w] >= cols) {
        result[w] = -1;
      }
    }
    return result;
  }

  /**
   * Execute a single phase of the algorithm. A phase of the Hungarian algorithm
   * consists of building a set of committed workers and a set of committed jobs
   * from a root unmatched worker by following alternating unmatched/matched
   * zero-slack edges. If an unmatched job is encountered, then an augmenting
   * path has been found and the matching is grown. If the connected zero-slack
   * edges have been exhausted, the labels of committed workers are increased by
   * the minimum slack among committed workers and non-committed jobs to create
   * more zero-slack edges (the labels of committed jobs are simultaneously
   * decreased by the same amount in order to maintain a feasible labeling).
   * <p>
   * 
   * The runtime of a single phase of the algorithm is O(n^2), where n is the
   * dimension of the internal square cost matrix, since each edge is visited at
   * most once and since increasing the labeling is accomplished in time O(n) by
   * maintaining the minimum slack values among non-committed jobs. When a phase
   * completes, the matching will have increased in size.
   */
  protected void executePhase() {
    while (true) {
      int minSlackWorker = -1, minSlackJob = -1;
      double minSlackValue = Double.POSITIVE_INFINITY;
      for (int j = 0; j < dim; j++) {
        if (parentWorkerByCommittedJob[j] == -1) {
          if (minSlackValueByJob[j] < minSlackValue) {
            minSlackValue = minSlackValueByJob[j];
            minSlackWorker = minSlackWorkerByJob[j];
            minSlackJob = j;
          }
        }
      }
      if (minSlackValue > 0) {
        updateLabeling(minSlackValue);
      }
      parentWorkerByCommittedJob[minSlackJob] = minSlackWorker;
      if (matchWorkerByJob[minSlackJob] == -1) {
        /*
         * An augmenting path has been found.
         */
        int committedJob = minSlackJob;
        int parentWorker = parentWorkerByCommittedJob[committedJob];
        while (true) {
          int temp = matchJobByWorker[parentWorker];
          match(parentWorker, committedJob);
          committedJob = temp;
          if (committedJob == -1) {
            break;
          }
          parentWorker = parentWorkerByCommittedJob[committedJob];
        }
        return;
      } else {
        /*
         * Update slack values since we increased the size of the committed
         * workers set.
         */
        int worker = matchWorkerByJob[minSlackJob];
        committedWorkers[worker] = true;
        for (int j = 0; j < dim; j++) {
          if (parentWorkerByCommittedJob[j] == -1) {
            double slack = costMatrix[worker][j] - labelByWorker[worker]
                - labelByJob[j];
            if (minSlackValueByJob[j] > slack) {
              minSlackValueByJob[j] = slack;
              minSlackWorkerByJob[j] = worker;
            }
          }
        }
      }
    }
  }

  /**
   * 
   * @return the first unmatched worker or {@link #dim} if none.
   */
  protected int fetchUnmatchedWorker() {
    int w;
    for (w = 0; w < dim; w++) {
      if (matchJobByWorker[w] == -1) {
        break;
      }
    }
    return w;
  }

  /**
   * Find a valid matching by greedily selecting among zero-cost matchings. This
   * is a heuristic to jump-start the augmentation algorithm.
   */
  protected void greedyMatch() {
    for (int w = 0; w < dim; w++) {
      for (int j = 0; j < dim; j++) {
        if (matchJobByWorker[w] == -1 && matchWorkerByJob[j] == -1
            && costMatrix[w][j] - labelByWorker[w] - labelByJob[j] == 0) {
          match(w, j);
        }
      }
    }
  }

  /**
   * Initialize the next phase of the algorithm by clearing the committed
   * workers and jobs sets and by initializing the slack arrays to the values
   * corresponding to the specified root worker.
   * 
   * @param w
   *          the worker at which to root the next phase.
   */
  protected void initializePhase(int w) {
    Arrays.fill(committedWorkers, false);
    Arrays.fill(parentWorkerByCommittedJob, -1);
    committedWorkers[w] = true;
    for (int j = 0; j < dim; j++) {
      minSlackValueByJob[j] = costMatrix[w][j] - labelByWorker[w]
          - labelByJob[j];
      minSlackWorkerByJob[j] = w;
    }
  }

  /**
   * Helper method to record a matching between worker w and job j.
   */
  protected void match(int w, int j) {
    matchJobByWorker[w] = j;
    matchWorkerByJob[j] = w;
  }

  /**
   * Reduce the cost matrix by subtracting the smallest element of each row from
   * all elements of the row as well as the smallest element of each column from
   * all elements of the column. Note that an optimal assignment for a reduced
   * cost matrix is optimal for the original cost matrix.
   */
  protected void reduce() {
    for (int w = 0; w < dim; w++) {
      double min = Double.POSITIVE_INFINITY;
      for (int j = 0; j < dim; j++) {
        if (costMatrix[w][j] < min) {
          min = costMatrix[w][j];
        }
      }
      for (int j = 0; j < dim; j++) {
        costMatrix[w][j] -= min;
      }
    }
    double[] min = new double[dim];
    for (int j = 0; j < dim; j++) {
      min[j] = Double.POSITIVE_INFINITY;
    }
    for (int w = 0; w < dim; w++) {
      for (int j = 0; j < dim; j++) {
        if (costMatrix[w][j] < min[j]) {
          min[j] = costMatrix[w][j];
        }
      }
    }
    for (int w = 0; w < dim; w++) {
      for (int j = 0; j < dim; j++) {
        costMatrix[w][j] -= min[j];
      }
    }
  }

  /**
   * Update labels with the specified slack by adding the slack value for
   * committed workers and by subtracting the slack value for committed jobs. In
   * addition, update the minimum slack values appropriately.
   */
  protected void updateLabeling(double slack) {
    for (int w = 0; w < dim; w++) {
      if (committedWorkers[w]) {
        labelByWorker[w] += slack;
      }
    }
    for (int j = 0; j < dim; j++) {
      if (parentWorkerByCommittedJob[j] != -1) {
        labelByJob[j] -= slack;
      } else {
        minSlackValueByJob[j] -= slack;
      }
    }
  }
  public static void main(String[] args)
  {
      Random rand = new Random(980980987);
      double[][] costMatrix = new double[17][16];
      for(int y=0;y<costMatrix[0].length;y++)
          for(int x=0;x<costMatrix.length;x++)
              costMatrix[x][y] = rand.nextDouble( ) + Double.MAX_VALUE;
      costMatrix[0][13] = 3.5;
      costMatrix[1][11] = 3;
      costMatrix[2][0] = 1;
      costMatrix[3][12] = 3;
      costMatrix[6][1] = 2;
      
      HungarianAlgorithm alg = new HungarianAlgorithm( costMatrix );
      int[] array = IntStream.rangeClosed(0, Math.max(costMatrix.length,costMatrix[0].length)).toArray();
      
      System.out.println(Arrays.toString( array ));
      System.out.println(Arrays.toString( alg.execute( ) ));
  }
}