using Android.App;
using Android.Content.Res;
using Android.OS;
using Android.Runtime;
using Android.Views;
using AndroidX.AppCompat.App;
using AndroidX.AppCompat.Widget;
using Google.Android.Material.FloatingActionButton;
using Google.Android.Material.Snackbar;
using SkiaSharp;
using SkiaSharp.Views.Android;
using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Text;

namespace PDF
{

    public class trailer
    {
        public Dictionary<string, string> Entries = new Dictionary<string, string>();
    }

    public class document_catalog
    {
        public Dictionary<string, string> Entries = new Dictionary<string, string>();
    }


    public class Pages
    {
        public Dictionary<string, string> Entries = new Dictionary<string, string>();
        public List<Pages> pages_tree_children_node_list = new List<Pages>();
        public Page page_leaf_node = new Page();
        public string context = new string("");
    }

    public class Page
    {
        public Dictionary<string, string> Entries = new Dictionary<string, string>();
        public string content;
    }

    public class textOperators
    {
        public string scn;
        public string Tf;
        public string Tm;
    
    }

    public class PDFCrossReferences
    {
        public string content;
        public MemoryStream memory_stream = new MemoryStream();
        public List<string> xref = new List<string>();

        private List<long> Positions = new List<long>();
        private List<int> Revisions = new List<int>();

        public long GetObjectPosition(int ObjectIndex) => Positions[ObjectIndex];

        public int GetObjectRevision(int ObjectIndex) => Revisions[ObjectIndex];

        public PDFCrossReferences(Stream stream)
        {
            var watch = System.Diagnostics.Stopwatch.StartNew();

            stream.CopyTo(this.memory_stream);
            this.memory_stream.Position = 0;
            this.content = Encoding.ASCII.GetString(this.memory_stream.ToArray());

            watch.Stop();
            var elapsedMs = watch.ElapsedMilliseconds;
            Console.WriteLine("==========ElapsedMilliseconds[to String]=====");
            Console.WriteLine(elapsedMs);
            Console.WriteLine(memory_stream.CanSeek);

            string trailer_string = read_trailer(content);
            string startxref = read_int(trailer_string, "startxref");
            this.xref = read_xref(content, int.Parse(startxref), read_length(content, int.Parse(startxref)));
            make_position();
        }

        public void make_position()
        {
            this.Positions = make_position(this.xref);
        }
        public void make_revision()
        {
            this.Revisions = make_revision(this.xref);
        }

        public List<long> make_position(List<string> xref)
        {
            List<long> position = new List<long>();
            for (int i = 0; i < xref.Count; i++)
            {
                position.Add(long.Parse(xref[i].Split(" ")[0]));
            }

            return position;
        }

        public List<int> make_revision(List<string> xref)
        {
            List<int> revision = new List<int>();
            for (int i = 0; i < xref.Count; i++)
            {
                revision.Add(int.Parse(xref[i].Split(" ")[1]));
            }

            return revision;
        }

        public string read_int(string content, string label)
        {
            string result = "";
            int entry_position = content.IndexOf(label) + label.Length;
            while (true)
            {
                result = result + content[entry_position];
                entry_position++;

                if (content[entry_position] == '\r' || content[entry_position] == '/' || content[entry_position] == '\n')
                {
                    break;
                }
            }

            return result;
        }



        public string read_length(string content, int index)
        {
            string result = "";
            int entry_position = index + 6; // +6 jump the "xref/r/n" length
            while (true)
            {
                if (content[entry_position] == '\r' || content[entry_position] == '\n')
                {
                    break;
                }
                result = result + content[entry_position];
                entry_position++;
            }

            return result;
        }

        public string read_xref_line(string content, int index)
        {
            string result = "";
            int entry_position = index;
            while (true)
            {
                if (content[entry_position] == '\r' || content[entry_position] == '\n')
                {
                    break;
                }
                result = result + content[entry_position];
                entry_position++;
            }

            return result;
        }

        public string read_obj_index(string content, string label)
        {
            string result = "";
            int entry_position = content.IndexOf(label) + label.Length;
            while (content[entry_position] != 'R')
            {
                result = result + content[entry_position];
                entry_position++;
            }

            return result + 'R';
        }
        public string read_trailer(string content)
        {
            string result = "";
            result = content.Split("trailer")[1];// time = 0.127s 
            return result;
        }

        public List<string> read_xref(string content, int index, string xref_length)
        {

            List<string> xref_list = new List<string>();

            int entry_position = index + 6 + xref_length.Length + 2;

            int length = int.Parse(xref_length.Split(" ")[1]);

            for (int i = 0; i < length; i++)
            {
                string xref_line = read_xref_line(content, entry_position);
                entry_position = entry_position + xref_line.Length + 2; //+2 is to jump the "/r/n"
                xref_list.Add(xref_line);
            }

            return xref_list;
        }

        public string read_array(string content, string label)
        {
            string result = "";
            int entry_position = content.IndexOf(label) + label.Length;
            while (content[entry_position] != ']')
            {
                result = result + content[entry_position];
                entry_position++;
            }

            return result + ']';
        }

        public string read_obj(string content, int index)
        {
            string result = "";

            int entry_position = index;


            while (true)
            {
                if (content[entry_position] == 'e' && content[entry_position + 1] == 'n' && content[entry_position + 2] == 'd' && content[entry_position + 3] == 'o' && content[entry_position + 4] == 'b' && content[entry_position + 5] == 'j')
                {
                    break;
                }

                result = result + content[entry_position];
                entry_position++;
            }

            return result;

        }

        public string read_string(string content, string label)
        {
            string result = "";
            int entry_position = content.IndexOf(label) + label.Length;

            while (true)
            {
                result = result + content[entry_position];
                entry_position++;

                if (content[entry_position] == '\r'  || content[entry_position] == '/' || content[entry_position] == '\n')
                {
                    break;
                }
            }

            return result;
        }

        public string read_type(string content, int index)
        {

            int line_number = int.Parse(this.xref[index].Split(" ")[0]);
            string content_object = read_obj(content, line_number);
            string type = read_string(content_object, "/Type");

            return type;
        }


        public PagesTreeNode make_pages(string content, List<string> xref, int object_index)
        {

            PagesTreeNode pages_tree_node = new PagesTreeNode();

            int line_number = int.Parse(xref[object_index].Split(" ")[0]);

            string pages_object = this.read_obj(content, line_number);

            string type = this.read_string(pages_object, "/Type");
            string count = this.read_int(pages_object, "/Count");
            string Kids = this.read_array(pages_object, "/Kids");


            if (type.Contains("/Pages") && pages_object.Contains("/Kids"))
            {
                pages_tree_node.Type = type;
                pages_tree_node.Count = int.Parse(count);
                pages_tree_node.Kids = Kids;

                Kids = Kids.Replace(" 0 ", " ");
                Kids = Kids.Replace("[", "");
                Kids = Kids.Replace("]", "");
                for (int i = 0; i < Kids.Split("R").Length; i++)
                {
                    string kids_index = Kids.Split("R")[i];

                    if (kids_index.Replace(" ", "").Length > 0)
                    {
                        pages_tree_node.pages_children_list.Add(make_pages(content, xref, int.Parse(kids_index)));
                    }

                }
            }
            else
            {
                PagesTreeNode pages_end = new PagesTreeNode();
                pages_end.Type = "Page";
                pages_tree_node.pages_children_list.Add(pages_end);
            }
            return pages_tree_node;
        }



        public string clean_front_empty_space(string content)
        {
            while (content[0] == ' ')
            {
                content = content.Remove(0, 1);
            }

            return content;
        }

    }

    public class PDFTrailer
    {
        public PDFCrossReferences CrossReferences;

        public int Size;
        public int RootIndex;
        public int InfoIndex;

        public PDFObject Root;
        public PDFObject Info;

        private void Initialize(MemoryStream PDFStream)
        {
            PDFStream.Position = CrossReferences.GetObjectPosition(RootIndex);
            Root = PDFObject.Create(PDFStream, CrossReferences, RootIndex);

            //PDFStream.Position = CrossReferences.GetObjectPosition(InfoIndex);
            //Info = PDFObject.Create(PDFStream, CrossReferences);
        }

        public PDFTrailer(MemoryStream ObjectStream, PDFCrossReferences References)
        {
            CrossReferences = References;

            string trailer_string = CrossReferences.read_trailer(CrossReferences.content);

            string size = CrossReferences.read_int(trailer_string, "/Size");
            string root = CrossReferences.read_obj_index(trailer_string, "/Root");
            string info = CrossReferences.read_obj_index(trailer_string, "/Info");

            root = CrossReferences.clean_front_empty_space(root);
            info = CrossReferences.clean_front_empty_space(info);


            this.Size = int.Parse(size);
            this.RootIndex = int.Parse(root.Split(" ")[0]);
            this.InfoIndex = int.Parse(info.Split(" ")[0]);

            Initialize(ObjectStream);
        }


    }

    public abstract class PDFObject
    {
        protected long Position;
        protected PDFCrossReferences CrossReferences;

        public PDFObject(MemoryStream PDFStream, PDFCrossReferences References)
        {
            CrossReferences = References;
        }

        public static PDFObject Create(MemoryStream PDFStream, PDFCrossReferences References, int ObjectIndex)
        {

            //Get Type;
            string Type = References.read_type(References.content, ObjectIndex);

            switch (Type)
            {
                case "/Catalog":
                    return new PDFCatalog(PDFStream, References);
                case "/Pages":
                    return new PDFPages(PDFStream, References);
                case "/Page":
                    return new PDFPage(PDFStream, References);

                default:
                    return null;
            }
        }
    }

    public class PDFCatalog : PDFObject
    {
        public int OutlinesIndex;
        public int PagesIndex;
        public string Type;

        public PDFPages Pages;

        private void Initialize(MemoryStream PDFStream)
        {
            //PDFStream.Position = CrossReferences.GetObjectPosition(PagesIndex);
            Pages = (PDFPages)PDFObject.Create(PDFStream, CrossReferences, PagesIndex);

        }

        public PDFCatalog(MemoryStream PDFStream, PDFCrossReferences References) : base(PDFStream, References)
        {
            int line_number = int.Parse(References.xref[1].Split(" ")[0]);
            string document_catalog_object = References.read_obj(References.content, line_number);

            string type = References.read_string(document_catalog_object, "/Type");
            string outlines = References.read_obj_index(document_catalog_object, "/Outlines");
            string pages = References.read_obj_index(document_catalog_object, "/Pages");


            outlines = References.clean_front_empty_space(outlines);
            pages = References.clean_front_empty_space(pages);

            this.Type = type;
            this.OutlinesIndex = int.Parse(outlines.Split(" ")[0]);
            this.PagesIndex = int.Parse(pages.Split(" ")[0]);

            Initialize(PDFStream);
        }
    }
    public class PagesTreeNode
    {
        public string Type;
        public int Count;
        public string Kids;

        public List<PagesTreeNode> pages_children_list = new List<PagesTreeNode>();

    }

    public class PDFPages : PDFObject
    {
        public PagesTreeNode page_tree_node = new PagesTreeNode();

        public PDFPages(MemoryStream PDFStream, PDFCrossReferences References) : base(PDFStream, References)
        {
            this.page_tree_node = References.make_pages(References.content, References.xref, 2);
        }



    }

    public class PDFPage : PDFObject
    {
        public PDFPage(MemoryStream PDFStream, PDFCrossReferences References) : base(PDFStream, References)
        {

        }
    }

    [Activity(Label = "@string/app_name", Theme = "@style/AppTheme.NoActionBar", MainLauncher = true)]
    public class MainActivity : AppCompatActivity
    {
        protected override void OnCreate(Bundle savedInstanceState)
        {
            base.OnCreate(savedInstanceState);
            Xamarin.Essentials.Platform.Init(this, savedInstanceState);
            SetContentView(Resource.Layout.activity_main);

            Toolbar toolbar = FindViewById<Toolbar>(Resource.Id.toolbar);
            SetSupportActionBar(toolbar);

            FloatingActionButton fab = FindViewById<FloatingActionButton>(Resource.Id.fab);
            fab.Click += FabOnClick;


            SKCanvasView canvasView = FindViewById<SKCanvasView>(Resource.Id.canvasView);
            canvasView.PaintSurface += OnPaintSurface;

        }

        private void OnPaintSurface(object sender, SKPaintSurfaceEventArgs e)
        {
            float CanvasScale = ((sender as SKCanvasView).Width - 12) / 794F;

            SKCanvas canvas = e.Surface.Canvas;

            canvas.Save();
            canvas.Translate(6, 6);
            canvas.Scale(CanvasScale, CanvasScale);

            var watch = System.Diagnostics.Stopwatch.StartNew();
            AssetManager assets = this.Assets;
            Stream stream = assets.Open("sample_2.pdf");

            watch.Stop();
            var elapsedMs = watch.ElapsedMilliseconds;
            Console.WriteLine("==========ElapsedMilliseconds[assets.Open]=====");
            Console.WriteLine(elapsedMs);
            Console.WriteLine(stream.CanSeek);

            MemoryStream memory_stream = new MemoryStream();
            stream.CopyTo(memory_stream);
            string content = Encoding.ASCII.GetString(memory_stream.ToArray());

            string trailer_string = read_trailer(content);
            //Console.WriteLine("=================startxref=================");
            //Console.WriteLine(trailer_string);

            string startxref = read_int(trailer_string, "startxref");



            Console.WriteLine(startxref);
            watch = System.Diagnostics.Stopwatch.StartNew();
            Dictionary<string, string> xref = read_xref(content, int.Parse(startxref), read_length(content, int.Parse(startxref)));

            watch.Stop();
            elapsedMs = watch.ElapsedMilliseconds;

            Console.WriteLine("==========ElapsedMilliseconds[read_xref]=====");
            Console.WriteLine(elapsedMs);

            /*
            int line_number = int.Parse(xref["6942"].Split(" ")[0]);

            string pages_object = read_obj(content, line_number);
            int stream_start = line_number + search_position_from_content(pages_object, "stream") + "stream".Length+2;//2 is for "/r/n"
            int stream_end = line_number + search_position_from_content(pages_object, "endstream")-2; //2 is for "/r/n"
            int length_stream = stream_end - stream_start;

            memory_stream.Position = stream_start;

            byte[] byte_stream = new byte[length_stream];

            
            memory_stream.Read(byte_stream, 0, length_stream);

            string sub_string = content.Substring(stream_start, length_stream);
            Console.WriteLine("=========sub_string============");
            Console.WriteLine(sub_string);
            // Convert a C# string to a byte array  
            byte[] bytes = Encoding.Default.GetBytes(sub_string);
            Console.WriteLine(bytes.Length);

            //jump 2 bytes of the stream
            byte[] cutinput = new byte[byte_stream.Length - 2];
            Array.Copy(byte_stream, 2, cutinput, 0, cutinput.Length);

            MemoryStream stream_output = new MemoryStream();
            using (MemoryStream compressStream = new MemoryStream(cutinput))
            using (DeflateStream decompressor = new DeflateStream(compressStream, CompressionMode.Decompress))
                decompressor.CopyTo(stream_output);
            string output_result = Encoding.Default.GetString(stream_output.ToArray());

            //*/

            //string output_result = read_content( memory_stream, content, xref, "6942");

            //Console.WriteLine("==========output_result[start]===========");
            //Console.WriteLine(output_result);
            //Console.WriteLine("==========output_result[end]===========");



            
            
            Pages complete_pages = make_pages( memory_stream, content, xref, "2");
            //Pages complete_pages = make_pages( memory_stream, content, xref, "16");
            //*/


            //Define the font state (Tf).
            //Position the text cursor(Td).
            //“Paint” the text onto the page(Tj).
            //< a > < b > < c > < d > < e > < f > Tm:  Manually define the text matrix.

            string stream_instruction = "BT \n " +          //begin
                "/CS0 cs 0 0 0  scn \n" +                   //1. scn
                "/GS0 gs \n " +                 
                "/TT0 1 Tf \n " +                           //2. Tf
                "9.96 0 0 9.96 72.024 745.92 Tm \n " +      //3. Tm
                "()Tj \n " +                                //4. Tj
                "0.6 0.6 0.6  scn \n " +                    //1. scn
                "24.4 - 71.082 Td \n" +                     //2. Td
                "()Tj \n" +                                 //3. Tj
                "0 0 0  scn \n" +                           //1. scn
                "/ TT1 1 Tf \n" +                           //2. Tf
                "0.021 Tc 48 0 0 48 194.3 623.74 Tm \n" +   //3. Tm
                "[(O)1(pen)1()1(XM)2(L)]TJ \n" +            //4. TJ
                "0 Tc 5.78 0 Td \n" +                       //5. Td
                "()Tj \n" +                                 //6. Tj
                "0.02 Tc - 4.572 - 1.215 Td \n" +           //7. Td
                "[(P)1(a)1(p) - 3(e) - 1(r)]TJ \n" +        //8. TJ
                "0 Tc()Tj \n" +                             //9. Tj
                "0.02 Tc - 2.066 - 1.216 Td \n" +           //10. Td
                " [(Sp) - 1(e) - 3(c)1(ifi) - 3(c) - 2(a)1(t)1(io)]TJ \n" + //11. TJ
                "0 Tc 6.76 0 Td \n" +                       //12. Td
                "(n)Tj \n" +                                //13. Tj
                "0.734 0 Td  \n" +                          //14. Td
                "()Tj \n" +                                 //15. Tj
                "ET";                                       //0. ET

            string[] scn_array = get_scn(stream_instruction);

            Console.WriteLine("===========scn_array=============");
            for (int i = 0; i < scn_array.Length; i++)
            {
                Console.WriteLine(scn_array[i]);
            }


            textOperators text_operators = new textOperators();
            string[] scn_operations =  get_operation(scn_array[1]);

            for (int i = 0; i < scn_operations.Length; i++)
            {
                Console.WriteLine("============[one]============");
                Console.WriteLine(scn_operations[i]);
                string text_single_operatorion = scn_operations[i];

                if (text_single_operatorion.Contains("Tf"))
                {
                    text_operators.Tf = text_single_operatorion;
                }

                if (text_single_operatorion.Contains("Tm"))
                {
                    text_operators.Tm = text_single_operatorion;
                }



            }

            /*
            string text = "";                                

            visit_tree_node(complete_pages, ref text);

            AppCompatTextView text_view = FindViewById<AppCompatTextView>(Resource.Id.text_view);
            text_view.SetText(text.ToCharArray(), 0, text.Length);
            //*/

        }                                           

        public static Stream GenerateStreamFromString(string s)
        {
            var stream = new MemoryStream();
            var writer = new StreamWriter(stream);
            writer.Write(s);
            writer.Flush();
            stream.Position = 0;
            return stream;
        }

        public string[] get_scn(string stream_instruction)
        {
            string[] result;

            result = stream_instruction.Split("scn");

            for (int i = 0; i < result.Length; i++)
            {
                result[i] = result[i] + "scn \n";
            }

            return result;
        
        }

        string[] get_operation(string scn_block)
        {
            string[] result;

            result = scn_block.Split('\n');

            return result;
        }

        public string read_int(string content, string label)
        {
            string result = "";
            int entry_position = content.IndexOf(label) + label.Length + 1;

            while (true)
            {
                result = result + content[entry_position];
                entry_position++;
                Console.WriteLine(result);

                if (content[entry_position] == '\r'  || content[entry_position] == '/' || content[entry_position] == '\n')
                {
                    break;
                }
            }

            return result;
        }

        public string read_length(string content, int index)
        {
            string result = "";
            int entry_position = index + 6;
            while (true)
            {
                if (content[entry_position] == '\r' || content[entry_position] == '\n')
                {
                    break;
                }
                result = result + content[entry_position];
                entry_position++;
            }

            return result;
        }

        public int search_position_from_content(string content, string string_searched)
        {
            return content.IndexOf(string_searched); ;
        }


        public string read_xref_line(string content, int index)
        {
            string result = "";
            int entry_position = index;
            while (true)
            {
                if (content[entry_position] == '\r' || content[entry_position] == '\n')
                {
                    break;
                }
                result = result + content[entry_position];
                entry_position++;
            }

            return result;
        }

        public Dictionary<string, string> read_xref(string content, int index, string xref_length)
        {

            Dictionary<string, string> xref_dictionary = new Dictionary<string, string>();

            int entry_position = index + 6 + xref_length.Length + 2;

            int length = int.Parse(xref_length.Split(" ")[1]);

            for (int i = 0; i < length; i++)
            {
                string xref_line = read_xref_line(content, entry_position);
                entry_position = entry_position + xref_line.Length + 2;
                xref_dictionary.Add(i.ToString(), xref_line);
            }

            return xref_dictionary;
        }

        public string read_obj(string content, int index)
        {
            string result = "";

            int entry_position = index;


            while (true)
            {
                if (content[entry_position] == 'e' && content[entry_position + 1] == 'n' && content[entry_position + 2] == 'd' && content[entry_position + 3] == 'o' && content[entry_position + 4] == 'b' && content[entry_position + 5] == 'j')
                {
                    break;
                }

                result = result + content[entry_position];
                entry_position++;
            }

            return result;

        }

        public string read_stream(string content, int index)
        {
            string result = "";

            int entry_position = content.IndexOf("stream") + "stream".Length;


            while (true)
            {
                if (content[entry_position] == 'e' && content[entry_position + 1] == 'n' && content[entry_position + 2] == 'd' && content[entry_position + 3] == 's' && content[entry_position + 4] == 't' && content[entry_position + 5] == 'r' && content[entry_position + 6] == 'e' && content[entry_position + 7] == 'a' && content[entry_position + 8] == 'm')
                {
                    break;
                }

                result = result + content[entry_position];
                entry_position++;
            }

            return result;

        }

        public string read_height(string content, string label)
        {
            string result = "";
            int entry_position = content.IndexOf(label) + label.Length;
            while (content[entry_position] != 'T')
            {
                result = result + content[entry_position];
                entry_position++;
            }

            return result;
        }

        public string read_array(string content, string label)
        {
            string result = "";

            //Console.WriteLine("==========read_array===========");
            //Console.WriteLine(content);
            //Console.WriteLine(label);

            int entry_position = content.IndexOf(label) + label.Length;
            while (content[entry_position] != ']')
            {
                result = result + content[entry_position];
                entry_position++;
            }

            return result + ']';
        }

        public string read_string(string content, string label)
        {
            string result = "";
            int entry_position = content.IndexOf(label) + label.Length;
            Console.WriteLine("==========read_string===========");
            Console.WriteLine(content);
            Console.WriteLine(label);
            Console.WriteLine(entry_position);


            while (true)
            {
                result = result + content[entry_position];
                entry_position++;

                if (content[entry_position] == '\r'  || content[entry_position] == '\n' || content[entry_position] == '/')
                {
                    break;
                }
            }

            return result;
        }


        public string read_obj_index(string content, string label)
        {
            string result = "";
            int entry_position = content.IndexOf(label) + label.Length;
            while (content[entry_position] != 'R')
            {
                result = result + content[entry_position];
                entry_position++;
            }

            return result + 'R';
        }

        public string read_obj_id(ref string content)
        {
            string result = "";
            int entry_position = 0;
            while (content[entry_position] != 'o')
            {
                result = result + content[entry_position];
                entry_position++;
            }

            return result + "obj";
        }

        public string read_trailer(string content)
        {
            var watch = System.Diagnostics.Stopwatch.StartNew();
            string result = "";
            result = content.Split("trailer")[1];     // time = 127 
            //int index = content.IndexOf("trailer");     // time = 1434
            //result = content.Substring(content.IndexOf("trailer")); 
            watch.Stop();
            var elapsedMs = watch.ElapsedMilliseconds;
            Console.WriteLine("==========ElapsedMilliseconds[read_trailer]=====");
            Console.WriteLine(elapsedMs);
            //Console.WriteLine(index);

            return result;
        }

        public string read_text(string content)
        {
            string result = "";
            int entry_position = content.IndexOf("(") + "(".Length;
            while (content[entry_position] != ')')
            {
                result = result + content[entry_position];
                entry_position++;
            }

            return result;
        }

        public string read_text_position(string content)
        {
            string result = "";
            int entry_position = content.IndexOf("Td") - 1;
            while (content[entry_position] != '\n')
            {
                result = content[entry_position] + result;
                entry_position--;
            }

            return result;
        }


        public string read_pdf_line(MemoryStream memory_stream)
        {
            string line_result = "";

            memory_stream.Seek(38348, SeekOrigin.Begin);   // From 0, %PDF-1.7 \r\n
            //Console.WriteLine(memory_stream.Position);
            for (int i = 0; i < 20; i++)
            {
                line_result += (char)memory_stream.ReadByte();
                Console.WriteLine(line_result); // From 38338 xref \r\n 0 space 39
            }

            return line_result;
        }

        public string read_pdf_line_from_string(string content, int start_position, int length)
        {
            string line_result = "";

            //Console.WriteLine(memory_stream.Position);
            for (int i = 0; i < length; i++)
            {
                line_result += (char)content[start_position + i];
                Console.WriteLine(line_result); // From 38338 xref \r\n 0 space 39
            }

            return line_result;
        }

        public string clean_front_empty_space(string content)
        {
            while (content[0] == ' ')
            {
                content = content.Remove(0, 1);
            }

            return content;
        }
        public string read_stream_length(string content, string length_tag)
        {
            string result = "";
            int entry_position = content.IndexOf(length_tag) + length_tag.Length; // +6 jump the "xref/r/n" length
            while (true)
            {
                if (content[entry_position] == '>' || content[entry_position] == '/')
                {
                    break;
                }
                result = result + content[entry_position];
                entry_position++;
            }

            return result;

        }

        public string read_resources(string content, string label)
        {
            string result = "";

            int entry_position = content.IndexOf(label) + label.Length;
            int uncloseed_bracket = 0;

            while (true)
            {
                if (content[entry_position] == '<' && content[entry_position+1] == '<')
                {
                    uncloseed_bracket++;
                    result = result + "<<";
                    entry_position = entry_position + 2;
                    continue;
                    
                }

                if (content[entry_position] == '>' && content[entry_position + 1] == '>')
                {
                    result = result + ">>";
                    uncloseed_bracket--;
                    if (uncloseed_bracket == 0)
                    { 
                        break;
                    }
                    entry_position = entry_position + 2;
                    continue;
                }


                result = result + content[entry_position];
                entry_position++;
            }

            return result;

        }

        public string read_content(MemoryStream memory_stream, string content, Dictionary<string,string> xref, string object_index)
        {

            int line_number = int.Parse(xref[object_index].Split(" ")[0]);

            //Console.WriteLine("==========line_number==========");
            //Console.WriteLine(line_number);

            string pages_object = read_obj(content, line_number);

            string stream_length = read_stream_length(pages_object, "/Length");

            //Console.WriteLine(pages_object);
            //Console.WriteLine(stream_length);

            if (stream_length.Length >= "0 0 R".Length || int.Parse(stream_length) > 0)
            {

                int stream_start = line_number + search_position_from_content(pages_object, "stream") + "stream".Length;//2 is for "/r/n"
                int stream_end = 0;

                if (content[stream_start ] == '\n')
                {
                    stream_start = line_number + search_position_from_content(pages_object, "stream") + "stream".Length + 1;
                    stream_end = line_number + search_position_from_content(pages_object, "endstream") - 2; //2 is for "/r/n"
                }
                else if(content[stream_start ] == '\r')
                {
                    stream_start = line_number + search_position_from_content(pages_object, "stream") + "stream".Length + 2;
                    stream_end = line_number + search_position_from_content(pages_object, "endstream") - 2; //2 is for "/r/n"
                }

                
                int length_stream = stream_end - stream_start;

                memory_stream.Position = stream_start;

                byte[] byte_stream = new byte[length_stream];


                memory_stream.Read(byte_stream, 0, length_stream);

                //jump 2 bytes of the stream 
                byte[] cutinput = new byte[byte_stream.Length - 2];
                Array.Copy(byte_stream, 2, cutinput, 0, cutinput.Length);

                MemoryStream stream_output = new MemoryStream();
                using (MemoryStream compressStream = new MemoryStream(cutinput))
                using (DeflateStream decompressor = new DeflateStream(compressStream, CompressionMode.Decompress))
                    decompressor.CopyTo(stream_output);
                string output_result = Encoding.ASCII.GetString(stream_output.ToArray());

                Console.WriteLine("==========output_result[start]===========");
                Console.WriteLine(object_index);
                Console.WriteLine(output_result);
                Console.WriteLine("==========output_result[end]===========");

                return output_result;
            }

            return null;

        }



        public Pages make_pages(MemoryStream memory_stream, string content, Dictionary<string, string> xref, string object_index)
        {

            Pages pages = new Pages();
            object_index = object_index.Replace(" ", "");

            //Console.WriteLine("================make_pages[start]=====================");
            //Console.WriteLine(object_index);
            //Console.WriteLine("================make_pages[end]=====================");

            int line_number = int.Parse(xref[object_index].Split(" ")[0]);

            string pages_object = read_obj(content, line_number);
            string type = read_string(pages_object, "/Type");
            string Kids = read_array(pages_object, "/Kids");
            pages.Entries.Add("/Type", type);
            pages.Entries.Add("/Count", read_int(pages_object, "/Count"));
            pages.Entries.Add("/Kids", Kids);
            pages.context = pages_object;

            Kids = Kids.Replace(" 0 ", " ");
            Kids = Kids.Replace("[", "");
            Kids = Kids.Replace("]", "");

            Console.WriteLine("================[Pages][start]=====================");
            Console.WriteLine(type);
            Console.WriteLine("================[Pages][type]=====================");
            Console.WriteLine(pages_object);
            Console.WriteLine(Kids);
            Console.WriteLine("================[Pages][end]=====================");

            if (type.Contains("/Pages") && pages_object.Contains("/Kids"))
            {
                for (int i = 0; i < Kids.Split("R").Length; i++)
                {
                    string kids_index = Kids.Split("R")[i];
                    Console.WriteLine("================Kids[start]=====================");
                    Console.WriteLine(kids_index);
                    Console.WriteLine("================Kids[end]=====================");
                    if (kids_index.Replace(" ", "").Length > 0)
                    {
                        pages.pages_tree_children_node_list.Add(make_pages(memory_stream,content, xref, kids_index));
                    }
                }
            }
            else if (type.Contains("/Page"))
            {
                Console.WriteLine("================Kids[Stop]=====================");
                Pages pages_end = new Pages();
                pages_end.Entries.Add("/Kids", "None");

                Page leaf_node_page = new Page();
                leaf_node_page.Entries.Add("/Parent", read_obj_index(pages_object, "/Parent"));
                leaf_node_page.Entries.Add("/MediaBox", read_array(pages_object, "/MediaBox"));
                string content_object = read_obj_index(pages_object, "/Contents");
                content_object = clean_front_empty_space(content_object);
                leaf_node_page.Entries.Add("/Contents", content_object);
                string resources_object = read_resources(pages_object, "/Resources");
                leaf_node_page.Entries.Add("/Resources", resources_object);

                Console.WriteLine(leaf_node_page.Entries["/Parent"]);
                leaf_node_page.content = read_content(memory_stream, content, xref, content_object.Split(" ")[0]);
                Console.WriteLine(leaf_node_page.content);

                pages_end.page_leaf_node = leaf_node_page;
                pages.pages_tree_children_node_list.Add(pages_end);
            }
            return pages;
        }

        int counter = 0;
        public string visit_tree_node(Pages pages, ref string text)
        {
            if (pages.Entries.ContainsKey("/Type") && pages.Entries["/Type"].Contains("Pages"))
            {
                text += "====================[Pages][start]=====================\n";
            }

            if (pages.Entries.ContainsKey("/Type") && pages.Entries["/Type"] != null)
            {
                text += pages.Entries["/Type"] + "\n";
            }

            if (pages.Entries.ContainsKey("/Count") && pages.Entries["/Count"] != null)
            {
                text += "Count = " + pages.Entries["/Count"] + "\n";
            }

            if (pages.page_leaf_node != null && pages.page_leaf_node.Entries.ContainsKey("/Parent") && pages.page_leaf_node.Entries["/Parent"] != null)
            {
                text += "Parent = " + pages.page_leaf_node.Entries["/Parent"] + "\n";
            }

            if (pages.page_leaf_node != null && pages.page_leaf_node.Entries.ContainsKey("/MediaBox") && pages.page_leaf_node.Entries["/MediaBox"] != null)
            {
                text += "MediaBox = " + pages.page_leaf_node.Entries["/MediaBox"] + "\n";
            }

            if (pages.page_leaf_node != null && pages.page_leaf_node.Entries.ContainsKey("/Contents") && pages.page_leaf_node.Entries["/Contents"] != null)
            {
                text += "Contents = " + pages.page_leaf_node.Entries["/Contents"] + "\n";
            }

            if (pages.page_leaf_node.content != null && counter < 1)
            {
                text += "Contents = " + pages.page_leaf_node.content + "\n";
                counter++;
            }

            if (pages.context != null)
            {
                if (pages.context.Length > 10)
                {
                    text += "ID = " + pages.context.Substring(0, 10) + "\n";
                }
                else if (pages.context.Length <= 10 && pages.context.Length >= 7)
                {
                    text += "ID = " + pages.context.Substring(0, 6) + "\n";
                }

            }

            if (pages.Entries.ContainsKey("/Kids") && pages.Entries.ContainsKey("/Count") && pages.Entries["/Kids"] != null && pages.Entries["/Count"].Replace(" ", "") != "0")
            {
                text += "Kids = " + pages.Entries["/Kids"] + "\n";
            }


            for (int i = 0; i < pages.pages_tree_children_node_list.Count; i++)
            {
                if (pages.Entries.ContainsKey("/Type") && pages.Entries["/Type"].Contains("Page"))
                {
                    text += "====================[Page]=====================\n";
                }

                if (pages.Entries["/Kids"] != "None")
                {
                    visit_tree_node(pages.pages_tree_children_node_list[i], ref text);
                }
            }

            return text;
        }

        public static byte[] Decompress(byte[] data)
        {
            Console.WriteLine(data.Length);
            byte[] decompressedArray = null;
            try
            {
                using (MemoryStream decompressedStream = new MemoryStream())
                {
                    using (MemoryStream compressStream = new MemoryStream(data))
                    {
                        using (DeflateStream deflateStream = new DeflateStream(compressStream, CompressionMode.Decompress))
                        {
                            deflateStream.CopyTo(decompressedStream);
                        }
                    }
                    decompressedArray = decompressedStream.ToArray();
                }
            }
            catch (Exception exception)
            {
                Console.WriteLine(exception);
            }

            return decompressedArray;
        }


        public override bool OnCreateOptionsMenu(IMenu menu)
        {
            MenuInflater.Inflate(Resource.Menu.menu_main, menu);
            return true;
        }

        public override bool OnOptionsItemSelected(IMenuItem item)
        {
            int id = item.ItemId;
            if (id == Resource.Id.action_settings)
            {
                return true;
            }

            return base.OnOptionsItemSelected(item);
        }

        private void FabOnClick(object sender, EventArgs eventArgs)
        {
            View view = (View)sender;
            Snackbar.Make(view, "Replace with your own action", Snackbar.LengthLong)
                .SetAction("Action", (View.IOnClickListener)null).Show();
        }

        public override void OnRequestPermissionsResult(int requestCode, string[] permissions, [GeneratedEnum] Android.Content.PM.Permission[] grantResults)
        {
            Xamarin.Essentials.Platform.OnRequestPermissionsResult(requestCode, permissions, grantResults);

            base.OnRequestPermissionsResult(requestCode, permissions, grantResults);
        }
    }
}
