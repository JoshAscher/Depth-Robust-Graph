import java.util.*;
import java.io.*;

public class DRG
{
  static int n;
  static int delta;
  static double e;
  static double d;
  static DAG D;

  public static void main(String [] args) throws IOException
  {
    Scanner scan = new Scanner(System.in);

    //for now, these parameters are hardcoded
    /*System.out.print("n = ");
    n = scan.nextInt();
    System.out.print("r0 = ");
    double r0 = scan.nextDouble();
    */
    System.out.println("Desired depth robustness?");
    System.out.print("e = ");
    e = scan.nextDouble();
    System.out.print("d = ");
    d = scan.nextDouble();



    n=1000; int r_0=4;
    D = new DAG(n);
    double alpha = findParameters(e,d,r_0);
    double beta = 1-alpha;//simplification for now;

    delta = (int)Math.ceil(-2*(-alpha*Math.log(alpha)-(1-alpha)*Math.log(1-alpha))*Math.log(2)/(alpha*Math.log((2-alpha)/2))); //condition for delta outlined in Overleaf doc
    System.out.println("alpha = " + alpha + ". beta = " + beta + ". delta = " + delta);

    // baseExpander(n, alpha, beta, r_0); //construct base of graph
    // System.out.println("edges added from base expander: " + D.m);

    //algorithm 5.1
    int edgeAdded = 0;
    Random rand = new Random();
    for(int v = 1; v < n-1; v++)
    {
      for(int i = 1; i < Math.ceil(delta * Math.log(v+1)/Math.log(2)); i++)
      {
        int k = (int)Math.floor(rand.nextDouble(Math.log(v+1)));//the higher the vertex number,
        Edge e = new Edge(v - (int)Math.pow(2,k),v);            //the longer an edge can possibly be
        if(D.addEdge(e))
          edgeAdded++;//System.out.println("added " + e.toString() + ". length = " +  e.length);
        // else
        //   System.out.println(e.toString() + " is duplicate");
      }
    }

    // for(int i = 0; i<n;i++)
    //   System.out.println("inDeg("+i+") = " + D.inDeg[i]);

    System.out.println("edges added from alg: " + edgeAdded);
    System.out.println("total size: " + D.m);

    lazyValiantAttack();
    System.out.println("Longest path after deletion = " + findLongestPath());
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
  public static void lazyValiantAttack()
  {
    int b = 2;
    int depthG = n-2; //every vertex will be connected to its neighbor, so the initial depth is just the path from 1->2->3->...->998->999
    int numSSets = (int)Math.ceil(Math.log(depthG)/Math.log(b));
    ArrayList<Integer> [] S = (ArrayList<Integer>[]) new ArrayList [numSSets]; //sets of edge destinations

    int maxDiffBit = (int)Math.ceil(Math.log(n)/Math.log(b));
    //because each vertex is connected to its neighbor, the labelling descibed by Valiant is the same as the inital labelling
    for(int u=1;u<n;u++) //for every vertex
    {
      for(Edge s : D.adj[u]) //and every edge adjacent to said vertex
      {
        int length = s.destination-u;
        for(int i = 0; i<maxDiffBit;i++)
        {
          if(length >= Math.pow(2,maxDiffBit-i))//if the length of that edge is greater than or equal to 2^i,
          {                          //then the bit representation of the adjacent vertices differ in the ith bit(from the left)
            if(S[i]==null)
              S[i] = new ArrayList<Integer>();
            S[i].add(s.destination);
            // if(i==1)
            //   System.out.println(s.toString() + ": added " + s.destination + " to S["+i+"]");
            break;
          }
        }
      }
    }

    //map size of each S_i to i(use TreeSet to keep in sorted order of size)
    TreeMap<Integer, Integer> SSize = new TreeMap<Integer,Integer>();
    for(int i = 0; i<S.length;i++)
      if(S[i]!=null)
        SSize.put(S[i].size(),i);

    //print size of each S_i
    for(int i : SSize.keySet())
      System.out.println("S[" + SSize.get(i) + "] contains " + i + " vertices.");

    //create set(no duplicate) of vertices to remove by adding from the smallest sized S_i
    Set<Integer> toRemove = new TreeSet<Integer>();
    for(int i : SSize.keySet())
    {
      ArrayList<Integer> S_i = S[SSize.get(i)];
      int j = 0;
      while(toRemove.size()<=e*n && j<S_i.size())
      {
        toRemove.add(S_i.get(j));
        j++;
      }
    }

    //remove vertices
    for(int s : toRemove)
    {
      deleteVertex(s);
      //System.out.println("removed " + s);
    }


    System.out.println("toRemove.size: " + toRemove.size());
  }

  public static void deleteVertex(int v) //throws Exception
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
  }

   static int findLongestPath()
   {
     ArrayList<Edge> longestPath = new ArrayList<Edge>();
     int longestLength = 0;
     for(int source = 1; source<n-1; source++)
     {
       for(int dest = 0; dest<n;dest++)
       {
         int currLength = 0;
         ArrayList<Edge> currPath = new ArrayList<Edge>();
         D.dijkstras(source,dest);

         if(!D.marked[dest])//if no path from source to dest, skip
          continue;

         Stack<Integer> path = new Stack<Integer>();
         for (int x = dest; x != source; x = D.edgeTo[x])
         {
            path.push(x);
            currLength++;
            currPath.add(new Edge(x, D.edgeTo[x]));
         }

         int prevVertex = source;
         while(!path.empty())
         {
           int w = path.pop();
           prevVertex = w;
         }

         if(currLength>longestLength)
         {
          longestLength = currLength;
          longestPath = currPath;
         }
       }
     }

     // System.out.print("longest path: ");
     // for(Edge e : longestPath)
     //   System.out.print(e.toString() + " ");
     // System.out.println();

     return longestLength;
   }

   //get bit representation of integer(unused for now)
   public static String toBitString(int n)
	 {
		if(n==0)
			return "";
		char [] arr = new char[(int)(Math.log(n)/Math.log(2)+1)];
		for(int i = 0; i<arr.length;i++)
			arr[i] = '0';
		while(n!=0)
		{
			double d = Math.log(n)/Math.log(2);
			arr[arr.length-1-(int)d] = '1';
			n=n-(int)Math.pow(2,(int)d);
		}
		return new String(arr);
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
