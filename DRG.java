import java.util.*;
import java.io.*;

public class DRG
{
  static int n;
  static int delta;
  static double e;
  static double d;
  static DAG D;

  public static void main(String [] args) throws Exception
  {
    Scanner scan = new Scanner(System.in);

    //for now, these parameters are hardcoded
    //System.out.print("r0 = ");
    //double r0 = scan.nextDouble();
    
    System.out.print("n = ");
    n = scan.nextInt();
    System.out.println("Desired depth robustness?");
    System.out.print("e = ");
    e = scan.nextDouble();
    System.out.print("d = ");
    d = scan.nextDouble();

    n=10000; int r_0=4;
    D = new DAG(n);
    double alpha = findParameters(e,d,r_0);
    double beta = 1-alpha;//simplification for now;

    delta = (int)Math.ceil(-2*(-alpha*log(alpha)-(1-alpha)*log(1-alpha))*Math.log(2)/(alpha*log((2-alpha)/2))); //condition for delta outlined in Overleaf doc
    System.out.println("alpha = " + alpha + ". beta = " + beta + ". delta = " + delta);

    // baseExpander(n, alpha, beta, r_0); //construct base of graph
    // System.out.println("edges added from base expander: " + D.m);

    //algorithm 5.1
    int edgeAdded = 0;
    int duplicateEdges = 0;
    Random rand = new Random();
    for(int v = 1; v < n-1; v++)
    {
      for(int i = 1; i < Math.ceil(delta * log(v+1)); i++)
      {
        double k = rand.nextDouble(log(v+1));//the higher the vertex number,
        Edge e = new Edge(v - (int)Math.floor(Math.pow(2,k)),v);            //the longer an edge can possibly be
        if(D.addEdge(e))
        {
          edgeAdded++;
          //System.out.println("added " + e.toString() + ". length = " +  e.length);
        }
        else
          duplicateEdges++;
      }
    }

    //System.out.println("edges added from alg: " + edgeAdded);
    System.out.println("duplicateEdges from alg: " + duplicateEdges);
    System.out.println("total size: " + D.m);
    // for(int i = 0; i<n;i++)
    //   System.out.println("inDeg("+i+") = " + D.inDeg[i]);

    valiantAttack(2);
    //System.out.println("Longest path after deletion = " + findLongestPath());

  }

  public static void printGraph()
  {
    for(int i  = 0; i < n;i++)
      if(D.adj[i]!=null)
        for(Edge e : D.adj[i])
          System.out.println(e.toString());
  }

  //for now we assume beta = 1-alpha
  static double findParameters(double e, double d, int r_0)
  {
    double alpha  = 0;
    double currAlpha = 0;
    while(currAlpha<.5)
    {
      double e_max = (1-2*currAlpha)/(1+2*currAlpha);
      double d_max = 1-e/e_max;
      if(e < e_max && d<=d_max)
        alpha = currAlpha;
      currAlpha+=.001;
    }
    return alpha;
  }

  public static void baseExpander(int n, double a, double b, double r0)
  {
    for(int i = 1 ; i<n-1;i++)
      for(int j = 1 ; j<n-1;j++)
        if(j-i>0 && j-i<2*r0)
          D.addEdge(new Edge(i,j));
  }

  @SuppressWarnings("unchecked")
  public static void valiantAttack(int b) throws Exception
  {
    int depthG = n-2; //every vertex will be connected to its neighbor, so the initial depth is just the path from 1->2->3->...->998->999
    int numSSets = (int)Math.ceil(log(depthG)/log(b));
    ArrayList<Integer> [] S = (ArrayList<Integer>[]) new ArrayList [numSSets]; //sets of edge destinations

    int maxDiffBit = (int)Math.ceil(log(n)/log(b));
    //because each vertex is connected to its neighbor, the labelling descibed by Valiant is the same as the inital labelling
    for(int u=1;u<n;u++) //for every vertex
    {
      String uBits = toBitString(u, maxDiffBit);
      for(Edge s : D.adj[u]) //and every edge adjacent to said vertex
      {
        String vBits = toBitString(s.destination, maxDiffBit);  //v is the destination node of edge s
        for(int i = 0; i<maxDiffBit;i++)
        {
          if(uBits.charAt(i)!= vBits.charAt(i))//if bit representation of the adjacent vertices differ in the ith bit(from the left)
          {
            if(S[i]==null)
              S[i] = new ArrayList<Integer>();
            S[i].add(s.destination);
            break;
          }
        }
      }
    }

    //notes
    /*
    build histogram with both endpoints
    almost everywhere expander proof
    */

    //map size of each S_i to i(use TreeSet to keep in sorted order of size)
    TreeMap<Integer, Integer> SSize = new TreeMap<Integer,Integer>();
    for(int i = 0; i<S.length;i++)
      if(S[i]!=null)
        SSize.put(S[i].size(),i);

    //print size of each S_i
    //for(int i : SSize.keySet())
    //  System.out.println("S[" + SSize.get(i) + "] contains " + i + " vertices.");

    //create set(no duplicate) of vertices to remove by adding from the smallest sized S_i
    Set<Integer> removed = new TreeSet<Integer>();
    Scanner pause = new Scanner(System.in);
    for(int i : SSize.keySet())
    {
      ArrayList<Integer> S_i = S[SSize.get(i)];
      Collections.sort(S_i); //makes it easier to see which vertices are removed if they are sorted
      for(int j = 0;j<S_i.size() && removed.size()<e*n;j++)
      {
        if(removed.add(S_i.get(j))) //don't attempt to remove a vertex more than once
          deleteVertex(S_i.get(j));//System.out.println("removed " + deleteVertex(S_i.get(j)));
      }

      //for modulating total number of vertices removed(if not just doing e*n)
      //System.out.println("totalRemoved = " + removed.size() + ". longestPath = " + findLongestPath());
      // System.out.print("continue(Y/n)?");
      // if(!pause.next().equals("Y"))
      //   System.exit(0);
    }
    System.out.println("totalRemoved = " + removed.size() + ". longestPath = " + findLongestPath());
  }



  public static int deleteVertex(int v) throws Exception
  {
    if(v > n)
    {
      throw new RuntimeException("Vertex " + v +  "not in digragh");
    }

    ArrayList<Edge> toRemove = new ArrayList<Edge>();

    for(int i = 1; i<n-1;i++)
    {
      Edge e = new Edge(i,v);
      if(D.adj[i]!=null && D.adj[i].contains(e) && i!=v)
      {
        //System.out.println("removing edge " + i + v );
        D.adj[i].remove(e);
      }
    }

    for(Edge r : D.adj[v])
    {
      toRemove.add(r);
    }

    D.adj[v].removeAll(toRemove);

    return v;//return deleted vertex
  }

  //for implementing Valiant attack as is(unused)
  public static Edge deleteEdge(Edge e) throws Exception
  {
    int i = 0;
    while(true)
    {
      if(D.adj[i].contains(e))
        break;
      if(i>=n)
        throw new RuntimeException("edge " + e + " not in digragh");
      i++;
    }

    int source = e.source;
    int destination = e.destination;
    D.adj[source].remove(e);
    D.outDeg[source]-=1;
    D.inDeg[destination]-=1;

    return e; //return deleted edge
  }

  //longest path algs found online----------------------------------------------
  static void dfs(int node, int dp[], boolean visited[])
  {
    // Mark as visited
    visited[node] = true;

    // Traverse for all its children
    for (int i = 0; i < D.adj[node].size(); i++)
    {
      // If not visited
      if (!visited[D.adj[node].get(i).destination])
        dfs(D.adj[node].get(i).destination, dp, visited);

      // Store the max of the paths
      dp[node] = Math.max(dp[node], 1 + dp[D.adj[node].get(i).destination]);
    }
  }

  static  int findLongestPath()
  {
    // Dp array
    int[] dp = new int[n+1];

    // Visited array to know if the node
    // has been visited previously or not
    boolean[] visited = new boolean[n + 1];

    // Call DFS for every unvisited vertex
    for (int i = 1; i <= n; i++)
    {
      if (!visited[i])
        dfs(i, dp, visited);
    }

    int ans = 0;

    // Traverse and find the maximum of all dp[i]
    for (int i = 1; i <= n; i++)
    {
      ans = Math.max(ans, dp[i]);
    }
    return ans;
  }

 //get bit representation of integer with maxBit length
 public static String toBitString(int n, int maxBit)
 {
	if(n==0)
		return "";
	char [] arr = new char[maxBit];
	for(int i = 0; i<arr.length;i++)
		arr[i] = '0';
	while(n!=0)
	{
		double d = log(n);
		arr[arr.length-1-(int)d] = '1';
		n=n-(int)Math.pow(2,(int)d);
	}
	return new String(arr);
 }

 //convert to log_2
 public static double log(double n)
 {
   return Math.log(n)/Math.log(2);
 }

}//end EEGraph class------------------------------------------------------------

class DAG
{
  public final int v;
  public int m;
  public LinkedList<Edge>[] adj;
  public boolean marked[];
  public int distTo[];
  public int edgeTo[];
  public int inDeg [];
  public int outDeg [];


  public DAG(int v)
  {
    if (v < 0) throw new RuntimeException("Number of vertices must be nonnegative");
    this.v = v;
    this.m = 0;
    @SuppressWarnings("unchecked")
    LinkedList<Edge>[] temp =
    (LinkedList<Edge>[]) new LinkedList[2*v];
    adj = temp;
    for (int i = 1; i < 2*v; i++)
    adj[i] = new LinkedList<Edge>();
    inDeg = new int [v];
    outDeg = new int [v];
  }

  public boolean addEdge(Edge edge)
  {
    int from = edge.source;
    int to = edge.destination;

    //case where both are null
    if(adj[from] == null && adj[to] == null)
    {
      adj[from] = new LinkedList<Edge>();
      adj[from].add(edge);
      m++;
      inDeg[edge.destination] +=1;
      outDeg[edge.source] +=1;
      return true;
    }

    //case where to is null, and from does not contain edge
    if(adj[from] != null && !adj[to].contains(edge) && adj[to] == null )
    {
      adj[from].add(edge);
      m++;
      inDeg[edge.destination] +=1;
      outDeg[edge.source] +=1;
      return true;
    }

    //case where from is null, and to does not contain edge
    if(adj[from] == null && adj[to] != null && !adj[to].contains(new Edge(to, from)))
    {
      adj[from] = new LinkedList<Edge>();
      adj[from].add(edge);
      m++;
      inDeg[edge.destination] +=1;
      outDeg[edge.source] +=1;
      return true;
    }

    //case where neither is null, and neither contains edge
    if(adj[from] != null && !adj[from].contains(edge) && adj[to] != null && !adj[to].contains(new Edge(to, from)))
    {
      adj[from].add(edge);
      m++;
      inDeg[edge.destination] +=1;
      outDeg[edge.source] +=1;
      return true;
    }

    return false; //otherwise

  }

  public void dijkstras(int source, int destination)
  {
    marked = new boolean[this.v];
    distTo = new int[this.v];
    edgeTo = new int[this.v];


    for (int i = 0; i < v; i++)
    {
      distTo[i] = -1; //initialize distance to every vertex to be -1
      marked[i] = false; //and unmarked
    }

    distTo[source] = 0; //source has distance of 0, and is marked
    marked[source] = true;
    int nMarked = 1; //number of marked vertices is 1

    int current = source;
    while (nMarked < this.v) //while there are unmarked vertices
    {
      for (Edge w : adj[current]) //check every adjacent edge to source
      {
        if (distTo[current]+1 > distTo[w.destination]) //if the adding this edge increases the distance to the a vertex v,
        {
          distTo[w.destination] = distTo[current]+1; //update the distTo array for that vertex
          edgeTo[w.destination] = current; //and update the edgeTo array to note the longestPath
        }
      }

      int max = -1;
      current = -1;

      for(int i=0; i<distTo.length; i++)
      {
        if(marked[i])
          continue;
        if(distTo[i] > max)
        {
          max = distTo[i];
          current = i;
        }
      }

      if(current==-1) break;
      marked[current]=true;
      nMarked--;

    }
  }
}//end DAG class------------------------------------------------------------

class Edge
{
  public int source;
  public int destination;
  public int length;

  public Edge(int source, int destination)
  {
    this.source = source;
    this.destination = destination;
    this.length = destination-source;
  }
  @Override
  public String toString()
  {
    return source + "->" + destination;
  }

  public boolean equals(Object other)
  {
    if (other instanceof Edge)
    {
      Edge otherEdge = (Edge) other;
      return source == otherEdge.source && destination == otherEdge.destination;
    }
    return false;
  }
}//end Edge class---------------------------------------------------------------
