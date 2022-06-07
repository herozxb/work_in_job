using Android.App;
using Android.Content.Res;
using Android.OS;
using Android.Runtime;
using Android.Views;
using AndroidX.AppCompat.App;
using AndroidX.AppCompat.Widget;
using Google.Android.Material.FloatingActionButton;
using Google.Android.Material.Snackbar;
using ICSharpCode.SharpZipLib.Zip.Compression.Streams;
using SkiaSharp;
using SkiaSharp.Views.Android;
using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Text;
using System.Text.RegularExpressions;

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


    public class Tc
    {
        public float value = 0;
    }

    public class Tm
    {
        public SKMatrix Tm_matrix = new SKMatrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
        public float Tc = 0;
    }

    public class Td
    {
        public float td_x = 0;
        public float td_y = 0;
        public float Tc = 0;
    }


    public class textOperators
    {
        public string scn;
        public string Tf;
        public Tm Tm;
        public Td Td;
        public string Tj;
        public string TJ;

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

        public PDFCrossReferences(MemoryStream memory_stream)
        {
            var watch = System.Diagnostics.Stopwatch.StartNew();

            this.memory_stream = memory_stream;

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

            while (!int.TryParse(content[entry_position].ToString(), out _))
            {
                entry_position++;
            }

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
            int entry_position = index + 6; // +6 jump the "xref/r\n" length
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
                entry_position = entry_position + xref_line.Length + 2; //+2 is to jump the "/r\n"
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
            //Console.WriteLine("==========read_string===========");
            //Console.WriteLine(content);
            //Console.WriteLine(label);
            //Console.WriteLine(entry_position);

            int slash_count = 0;

            while (true)
            {
                if (content[entry_position] == '/')
                {
                    slash_count++;
                }

                if (content[entry_position] == '\r' || content[entry_position] == '\n' || content[entry_position] == '>' || slash_count > 1)
                {

                    break;
                }

                result = result + content[entry_position];
                entry_position++;


            }

            return result;
        }

        public string read_type(string content, int index)
        {

            int line_number = int.Parse(this.xref[index].Split(" ")[0]);
            string content_object = read_obj(content, line_number);
            string type = read_string(content_object, "/Type");

            //Console.WriteLine("===============================");
            Console.WriteLine(content_object);

            return type;
        }


        public PagesTreeNode make_pages(MemoryStream PDFStream, PDFCrossReferences References, int object_index)//string content, List<string> xref, int object_index)
        {
            string content = References.content;
            List<string> xref = References.xref;

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
                        pages_tree_node.pages_children_list.Add(make_pages(PDFStream, References, int.Parse(kids_index)));
                    }

                }
            }
            else
            {

                PDFPage pdf_page = (PDFPage)PDFObject.Create(PDFStream, References, object_index);
                pages_tree_node.page_tree_leaf_node = pdf_page;
            }
            return pages_tree_node;
        }

        public PageTreeLeafNode make_page(MemoryStream PDFStream, PDFCrossReferences References, int object_index)//string content, List<string> xref, int object_index)
        {
            string content = References.content;
            List<string> xref = References.xref;

            int line_number = int.Parse(xref[object_index].Split(" ")[0]);

            string pages_object = References.read_obj(content, line_number);

            PageTreeLeafNode leaf_node_page = new PageTreeLeafNode();
            leaf_node_page.Type = "PageLeafNode";

            Console.WriteLine("================Kids[Stop]=====================");
            if (pages_object.Contains("/Parent"))
            {
                leaf_node_page.Parent = References.read_obj_index(pages_object, "/Parent");
            }
            if (pages_object.Contains("/MediaBox"))
            {
                leaf_node_page.MediaBox = References.read_array(pages_object, "/MediaBox");
            }

            if (pages_object.Contains("/Contents"))
            {
                string content_object = References.read_obj_index(pages_object, "/Contents");
                content_object = References.clean_front_empty_space(content_object);
                leaf_node_page.Contents = content_object;
            }

            if (pages_object.Contains("/Rotate"))
            {
                leaf_node_page.Rotate = References.read_int(pages_object, "/Rotate");
            }

            if (pages_object.Contains("/Annots"))
            {
                leaf_node_page.Annots = References.read_obj_index(pages_object, "/Annots");
            }

            return leaf_node_page;
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
            Type = Type.Replace(" ", "");
            Type = Type.Replace(">", "");

            switch (Type)
            {
                case "/Catalog":
                    {
                        int RootIndex = ObjectIndex;
                        return new PDFCatalog(PDFStream, References, RootIndex);
                    }
                case "/Pages":
                    {
                        int PagesIndex = ObjectIndex;
                        return new PDFPages(PDFStream, References, PagesIndex);
                    }
                case "/Page":
                    {
                        int PageIndex = ObjectIndex;
                        return new PDFPage(PDFStream, References, PageIndex);
                    }

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
            this.Pages = (PDFPages)PDFObject.Create(PDFStream, CrossReferences, PagesIndex);

        }

        public PDFCatalog(MemoryStream PDFStream, PDFCrossReferences References, int PagesIndex) : base(PDFStream, References)
        {
            string page_line = References.xref[PagesIndex].Split(" ")[0];
            int line_number = int.Parse(page_line);
            string document_catalog_object = References.read_obj(References.content, line_number);

            string type = References.read_string(document_catalog_object, "/Type");
            string outlines = References.read_obj_index(document_catalog_object, "/Outlines");
            string pages = References.read_obj_index(document_catalog_object, "/Pages");


            outlines = References.clean_front_empty_space(outlines);
            pages = References.clean_front_empty_space(pages);

            this.Type = type;
            if (outlines.Contains("/Outlines"))
            {
                this.OutlinesIndex = int.Parse(outlines.Split(" ")[0]);
            }

            string pages_index = pages.Split(" ")[0];
            this.PagesIndex = int.Parse(pages_index);

            Initialize(PDFStream);
        }
    }
    public class PagesTreeNode
    {
        public string Type;
        public string Parent;
        public int Count;
        public string Kids;

        public List<PagesTreeNode> pages_children_list = new List<PagesTreeNode>();
        //public PageTreeLeafNode page_tree_leaf_node = new PageTreeLeafNode();
        public PDFPage page_tree_leaf_node;
    }

    public class PageTreeLeafNode
    {
        public string Type;
        public string Parent;
        public string LastModified;
        public string Resources;
        public string MediaBox;
        public string Contents;
        public string Rotate;
        public string Thumb;
        public string Dur;
        public string Trans;
        public string Annots;
        public string Metadata;

    }

    public class PDFPages : PDFObject
    {
        public PagesTreeNode pages_tree_node = new PagesTreeNode();

        public PDFPages(MemoryStream PDFStream, PDFCrossReferences References, int PagesIndex) : base(PDFStream, References)
        {
            this.pages_tree_node = References.make_pages(PDFStream, References, PagesIndex);
        }

    }

    public class PDFPage : PDFObject
    {
        public PageTreeLeafNode page_tree_leaf_node = new PageTreeLeafNode();
        public PDFPage(MemoryStream PDFStream, PDFCrossReferences References, int PageIndex) : base(PDFStream, References)
        {
            this.page_tree_leaf_node = References.make_page(PDFStream, References, PageIndex);
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


            /*
            
            AssetManager assets = this.Assets;
            Stream stream = assets.Open("sample_5.pdf");

            MemoryStream memory_stream = new MemoryStream();
            stream.CopyTo(memory_stream);


            
            PDFCrossReferences pdf_cross_reference = new PDFCrossReferences(memory_stream);

            PDFTrailer pdf_trailer_object = new PDFTrailer(memory_stream, pdf_cross_reference);

            PDFPages pdf_pages = ((PDFCatalog)(pdf_trailer_object.Root)).Pages;
            int i = 0;
            //*/


            AssetManager assets = this.Assets;
            Stream stream = assets.Open("sample_2.pdf");
            MemoryStream memory_stream = new MemoryStream();
            stream.CopyTo(memory_stream);
            string content = Encoding.ASCII.GetString(memory_stream.ToArray());

            string trailer_string = read_trailer(content);
            string size_in_trailer = read_int(trailer_string, "/Size");
            string root_in_trailer = read_obj_index(trailer_string, "/Root");
            string info_in_trailer = read_obj_index(trailer_string, "/Info");
            string startxref = read_int(trailer_string, "startxref");

            Dictionary<string, string> xref = read_xref(content, int.Parse(startxref), read_length(content, int.Parse(startxref)));

            int line_number = int.Parse(xref[clean_front_empty_space(root_in_trailer).Split(" ")[0]].Split(" ")[0]);
            string root_object = read_obj(content, line_number);
            string pages_start_index = read_obj_index(root_object, "/Pages");
            string output_result = read_content(memory_stream, content, xref, "6942"); //6942 is the start page,    378 is the font


            float page_width = (float)612.0;
            float page_height = (float)792.0;

            SKPaint paint = new SKPaint();
            paint.Style = SKPaintStyle.Stroke;
            paint.Color = new SKColor(0, 0, 0);
            paint.StrokeWidth = 0;
            canvas.DrawRect(0, 0, page_width, page_height, paint);

            canvas.Flush();
            canvas.Restore();


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


        public Tm make_tm(string content)
        {

            Tm Tm = new Tm();

            SKMatrix tm_matrix = new SKMatrix();


            if (content.Contains("Tc"))
            {

                string tc = content.Split("Tc")[0];
                string tm = content.Split("Tc")[1];
                tm = clean_front_empty_space(tm);

                tm_matrix.ScaleX = float.Parse(tm.Split(" ")[0]);
                tm_matrix.SkewX = float.Parse(tm.Split(" ")[1]);
                tm_matrix.SkewY = float.Parse(tm.Split(" ")[2]);
                tm_matrix.ScaleY = float.Parse(tm.Split(" ")[3]);
                tm_matrix.TransX = float.Parse(tm.Split(" ")[4]);
                tm_matrix.TransY = float.Parse(tm.Split(" ")[5]);

                Tm.Tm_matrix = tm_matrix;
                Tm.Tc = float.Parse(tc);



            }
            else if (content.Contains("Tm"))
            {
                string tm = clean_front_empty_space(content);

                tm_matrix.ScaleX = float.Parse(tm.Split(" ")[0]);
                tm_matrix.SkewX = float.Parse(tm.Split(" ")[1]);
                tm_matrix.SkewY = float.Parse(tm.Split(" ")[2]);
                tm_matrix.ScaleY = float.Parse(tm.Split(" ")[3]);
                tm_matrix.TransX = float.Parse(tm.Split(" ")[4]);
                tm_matrix.TransY = float.Parse(tm.Split(" ")[5]);

                Tm.Tm_matrix = tm_matrix;

            }

            return Tm;

        }

        public string make_TJ(string content,  SKPoint position, SKPaint paint, SKCanvas canvas, Tm Tm, Tc Tc, string widths)
        {
            string[] TJ_array = content.Split("(");

            
            for (int i = 0; i < TJ_array.Length; i++)
            {
                Console.WriteLine("============make_TJ===========");
                Console.WriteLine(TJ_array[i]);
                if (!TJ_array[i].Contains("[") && TJ_array[i].Contains(")") )
                {
                    string TJ_content = TJ_array[i].Split(")")[0];
                    string td_content = TJ_array[i].Split(")")[1];

                    if (TJ_content == "")
                    {
                        TJ_content = " ";
                    }

                    Console.WriteLine("==================TJ_content_DrawText.X[start]================");
                    Console.WriteLine("TJ_content:" + TJ_content);
                    Console.WriteLine("TJ_content length:"+TJ_content.Length.ToString());
                    Console.WriteLine(td_content);
                    Console.WriteLine(position.X);
                    Console.WriteLine("==================TJ_content_DrawText.X[end]================");
                    canvas.DrawText(TJ_content, position, paint);

                    if (!td_content.Contains("TJ"))
                    {
                        //td_content = td_content.Replace(" ", ""); 
                        string td_content_clean = "";
                        int td_index = 0;
                        while (td_index < td_content.Length)
                        {
                            if (td_content[td_index] == '-')
                            {
                                td_content_clean = td_content_clean + td_content[td_index];
                                td_index++;
                                while (td_content[td_index] == ' ')
                                {
                                    td_index++;
                                }

                            }
                            td_content_clean = td_content_clean + td_content[td_index];
                            td_index++;
                        }

                        Console.WriteLine(td_content_clean);
                        var matches = Regex.Matches(td_content_clean, @"-?[0-9]*\.?[0-9]+");
                        float td_x = float.Parse(matches[0].Value);
                        Console.WriteLine("==================position.X[start]================");
                        Console.WriteLine("pre td_x=" + td_x.ToString());
                        Console.WriteLine("pre X="+position.X.ToString());

                        float[] glyph_width = paint.GetGlyphWidths(TJ_content);

                        float TJ_width = 0;
                        for (int j = 0; j < glyph_width.Length; j++)
                        {
                            TJ_width = TJ_width + glyph_width[j];
                        }

                        float width_TJ = 0;

                        for (int k = 0; k < TJ_content.Length; k++)
                        {
                            int index_i = (int)TJ_content[k] - 32;
                            if (TJ_content[k] == ' ')
                            {
                                index_i = 32 - 32;
                            }
                            //Console.WriteLine(widths.Replace("[", "").Replace("]", "").Split(" ")[index_i]);
                            string char_width = widths.Replace("[", "").Replace("]", "").Split(" ")[index_i];
                            width_TJ = width_TJ + float.Parse(char_width) / 1000;
                        }


                        /*
                         * 
The equation
It looks like you attempt to interpret the equation

tx = ((w0 - Tj/1000) * Tfs + Tc + Tw) * Th
in glyph space instead of text space, at least the values you insert appear to indicate that. Instead interpret it in text space in a font-type agnostic manner.

In particular:

w0 - use the width of the respective character from the width array in the PDF font dictionary and divide it by 1000; I would expect a value of about .667 for the capital A.
Tj - use the value from the TJ operation array parameter, e.g. 120.
Tfs - use the font size from the graphics state which is the font size parameter from the relevant Tf operation, e.g. 10.
Tc - use the value from the graphics state which is the parameter from the relevant Tc or " operation.
Tw - use 0 or (in case of a single-byte character code 32) the value from the graphics state which is the parameter from the relevant Tw or " operation.
Th - use the value from the graphics state which is the parameter from the relevant Tz operation divided by 100.

                        // */

                        //position.X = position.X + (float)(td_x/ 1000 * 0 + Tc.value) * Tm.Tm_matrix.ScaleX  + TJ_width;

                        position.X = position.X + (width_TJ - td_x / 1000 + Tc.value) * Tm.Tm_matrix.ScaleX; 

                        Console.WriteLine("post X=" + position.X.ToString());
                        Console.WriteLine("TJ_width=" + TJ_width.ToString());
                        Console.WriteLine("==================position.X[end]================");

                    }



                }
            }

            String TJ_text = "";
            int index = 0;
            while (index < content.Length)
            {
                if (content[index] == '(')
                {
                    index++;
                    if (content[index] == ')')
                    {
                        continue;
                    }

                    TJ_text = TJ_text + content[index];
                    while (true)
                    {

                        index++;

                        if (content[index] == ')')
                        {
                            break;
                        }

                        TJ_text = TJ_text + content[index];
                    }
                }
                index++;
            }

            return TJ_text;
        }

        public string make_Tj(string content)
        {
            int start_text = content.IndexOf("(") + 1;
            int end_text = content.IndexOf(")");
            //Console.WriteLine(start_text);
            //Console.WriteLine(end_text);

            if (content.Contains("<"))
                return "";

            string Tj_content = content.Substring(start_text, end_text - start_text);
            Console.WriteLine(Tj_content);
            if (Tj_content == " ")
            {
                Console.WriteLine("===============empty Tj( )================");
                Console.WriteLine(Tj_content.Length);
            }

            return Tj_content;
        }

        public Td make_td(string content)
        {
            Td Td = new Td();
            Console.WriteLine("======make_td========");
            Console.WriteLine(content);
            if (content.Contains("Tc"))
            {
                string tc_content = content.Split("Tc")[0];
                string td_content = content.Split("Tc")[1];
                td_content = clean_front_empty_space(td_content);

                Console.WriteLine("================tc================");
                Console.WriteLine(td_content);

                //td_content = td_content.Replace(" ", ""); 
                string td_content_clean = "";
                int index = 0;
                while (index<td_content.Length)
                {
                    if (td_content[index] == '-')
                    {
                        td_content_clean = td_content_clean + td_content[index];
                        index++;
                        while (td_content[index] == ' ')
                        {
                            index++;
                        }

                    }
                    td_content_clean = td_content_clean + td_content[index];
                    index++;
                }

                Console.WriteLine(td_content_clean);
                var matches = Regex.Matches(td_content_clean, @"-?[0-9]*\.?[0-9]+");

                Td.Tc = float.Parse(tc_content.Replace(" ",""));
                Td.td_x = float.Parse(matches[0].Value);
                Td.td_y = float.Parse(matches[1].Value);
                Console.WriteLine(Td.td_x);
                Console.WriteLine(Td.td_y);
            }
            else
            {
                if (!content.Contains("-"))
                {
                    content = clean_front_empty_space(content);
                    Td.td_x = float.Parse(content.Split(" ")[0]);
                    Td.td_y = float.Parse(content.Split(" ")[1]);
                }
                else
                {
                    content = content.Replace(" ", "");
                    var matches = Regex.Matches(content, @"-?[0-9]*\.?[0-9]+");
                    Td.Tc = float.Parse("0");
                    Td.td_x = float.Parse(matches[0].Value);
                    Td.td_y = float.Parse(matches[1].Value);
                }

            }

            return Td;

        }

        //*/
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
            //Console.WriteLine("==========read_string===========");
            //Console.WriteLine(content);
            //Console.WriteLine(label);
            //Console.WriteLine(entry_position);

            int slash_count = 0;

            while (true)
            {
                if (content[entry_position] == '/')
                {
                    slash_count++;
                }

                if (content[entry_position] == '\r' || content[entry_position] == '\n' || content[entry_position] == '>' || slash_count > 1)
                {

                    break;
                }

                result = result + content[entry_position];
                entry_position++;


            }

            //Console.WriteLine(result);
            //Console.WriteLine("==========read_string[end]===========");

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
            int entry_position = content.IndexOf(length_tag) + length_tag.Length; // +6 jump the "xref/r\n" length
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

            string index_result = read_obj_index(content, label);

            if (index_result.Contains('<') || index_result.Contains('>'))
            {

                int entry_position = content.IndexOf(label) + label.Length;
                int uncloseed_bracket = 0;

                while (true)
                {
                    if (content[entry_position] == '<' && content[entry_position + 1] == '<')
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
            else if (index_result.Contains("R"))
            {
                return index_result;
            }

            result = "No Resources";

            return result;
        }

        public string read_content(MemoryStream memory_stream, string content, Dictionary<string, string> xref, string object_index)
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

                int stream_start = line_number + search_position_from_content(pages_object, "stream") + "stream".Length;//2 is for "/r\n"
                int stream_end = 0;

                if (content[stream_start] == '\n')
                {
                    stream_start = line_number + search_position_from_content(pages_object, "stream") + "stream".Length + 1;
                    stream_end = line_number + search_position_from_content(pages_object, "endstream") - 2; //2 is for "/r\n"
                }
                else if (content[stream_start] == '\r')
                {
                    stream_start = line_number + search_position_from_content(pages_object, "stream") + "stream".Length + 2;
                    stream_end = line_number + search_position_from_content(pages_object, "endstream") - 2; //2 is for "/r\n"
                }


                int length_stream = stream_end - stream_start;

                memory_stream.Position = stream_start;

                byte[] byte_stream = new byte[length_stream];


                memory_stream.Read(byte_stream, 0, length_stream);

                /*
                int start = 0;
                while ((byte_stream[start] == 0x0a) | (byte_stream[start] == 0x0d)) start++; // skip trailling cr, lf

                Console.WriteLine("========start=========");
                Console.WriteLine(start);


                byte[] tempdata = new byte[byte_stream.Length - start];
                Array.Copy(byte_stream, start, tempdata, 0, byte_stream.Length - start);

                MemoryStream msInput = new MemoryStream(byte_stream);
                MemoryStream msOutput = new MemoryStream();

                Console.WriteLine("====================msInput======================");
                Console.WriteLine(msInput.Length);
                Console.WriteLine(msInput.Position);
                try
                {
                    GZipStream decomp = new GZipStream(msInput, CompressionMode.Decompress);
                    decomp.CopyTo(msOutput);
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.Message);
                }

                string output_result = Encoding.ASCII.GetString(msOutput.ToArray());
                Console.WriteLine("====================msOutput======================");
                Console.WriteLine(msOutput.Length);
                Console.WriteLine(msOutput.Position);
                //*/


                var outputStream = new MemoryStream();
                using var compressedStream = new MemoryStream(byte_stream);
                using var inputStream = new InflaterInputStream(compressedStream);
                inputStream.CopyTo(outputStream);
                outputStream.Position = 0;
                string output_result = Encoding.Default.GetString(outputStream.ToArray());


                /*
                using (outputStream = new MemoryStream())
                {
                    using (var compressedDataStream = new MemoryStream(byte_stream))
                    {
                        // Remove the first two bytes to skip the header (it isn't recognized by the DeflateStream class)
                        compressedDataStream.ReadByte();
                        compressedDataStream.ReadByte();

                        var deflateStream = new DeflateStream(compressedDataStream, CompressionMode.Decompress, true);

                        var decompressedBuffer = new byte[1024];
                        int read;
                        while ((read = deflateStream.Read(decompressedBuffer, 0, decompressedBuffer.Length)) != 0)
                        {
                            outputStream.Write(decompressedBuffer, 0, read);
                        }
                        outputStream.Flush();
                        compressedDataStream.Close();
                    }
                    outputStream.Position = 0;
                    output_result = Encoding.Default.GetString(outputStream.ToArray());

                }
                //*/


                /*
                Console.WriteLine("====================new[start]======================");
                for (int i = 0; i < data.Length; i++)
                {
                    Console.Write(data[i]);
                }
                Console.WriteLine("====================new[end]======================");
                //*/


                /*
                //jump 2 bytes of the stream 
                byte[] cutinput = new byte[byte_stream.Length - 2];
                Array.Copy(byte_stream, 2, cutinput, 0, cutinput.Length);

                MemoryStream stream_output = new MemoryStream();
                using (MemoryStream compressStream = new MemoryStream(cutinput))
                using (DeflateStream decompressor = new DeflateStream(compressStream, CompressionMode.Decompress))
                    decompressor.CopyTo(stream_output);
                //string output_result = Encoding.ASCII.GetString(stream_output.ToArray());
                //*/

                //string output_result = "";
                Console.WriteLine("==========output_result[start][in_read_content]===========");
                Console.WriteLine(object_index);
                Console.WriteLine(output_result);
                Console.WriteLine("==========output_result[end][in_read_content]===========");

                return output_result;
            }

            return null;

        }

        public MemoryStream make_font(MemoryStream memory_stream, string content, Dictionary<string, string> xref, string object_index)
        {

            int line_number = int.Parse(xref[object_index].Split(" ")[0]);


            string pages_object = read_obj(content, line_number);

            string stream_length = read_stream_length(pages_object, "/Length");

            if (stream_length.Length >= "0 0 R".Length || int.Parse(stream_length) > 0)
            {

                int stream_start = line_number + search_position_from_content(pages_object, "stream") + "stream".Length;//2 is for "/r\n"
                int stream_end = 0;

                if (content[stream_start] == '\n')
                {
                    stream_start = line_number + search_position_from_content(pages_object, "stream") + "stream".Length + 1;
                    stream_end = line_number + search_position_from_content(pages_object, "endstream") - 2; //2 is for "/r\n"
                }
                else if (content[stream_start] == '\r')
                {
                    stream_start = line_number + search_position_from_content(pages_object, "stream") + "stream".Length + 2;
                    stream_end = line_number + search_position_from_content(pages_object, "endstream") - 2; //2 is for "/r\n"
                }


                int length_stream = stream_end - stream_start;

                memory_stream.Position = stream_start;

                byte[] byte_stream = new byte[length_stream];

                memory_stream.Read(byte_stream, 0, length_stream);

                var outputStream = new MemoryStream();
                using var compressedStream = new MemoryStream(byte_stream);
                using var inputStream = new InflaterInputStream(compressedStream);
                inputStream.CopyTo(outputStream);
                outputStream.Position = 0;

                return outputStream;

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
                        pages.pages_tree_children_node_list.Add(make_pages(memory_stream, content, xref, kids_index));
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

                //string resources_object = read_resources(pages_object, "/Resources");
                //leaf_node_page.Entries.Add("/Resources", resources_object);

                leaf_node_page.Entries.Add("/MediaBox", read_array(pages_object, "/MediaBox"));

                string content_object = read_obj_index(pages_object, "/Contents");
                content_object = clean_front_empty_space(content_object);
                leaf_node_page.Entries.Add("/Contents", content_object);

                //leaf_node_page.Entries.Add("/Rotate", read_int(pages_object, "/Rotate"));

                //leaf_node_page.Entries.Add("/Annots", read_obj_index(pages_object, "/Annots"));

                Console.WriteLine(leaf_node_page.Entries["/Parent"]);
                //leaf_node_page.content = read_content(memory_stream, content, xref, content_object.Split(" ")[0]);
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

            if (pages.page_leaf_node != null && pages.page_leaf_node.Entries.ContainsKey("/Resources") && pages.page_leaf_node.Entries["/Resources"] != null)
            {
                text += "Resources = " + pages.page_leaf_node.Entries["/Resources"] + "\n";
            }


            if (pages.page_leaf_node != null && pages.page_leaf_node.Entries.ContainsKey("/MediaBox") && pages.page_leaf_node.Entries["/MediaBox"] != null)
            {
                text += "MediaBox = " + pages.page_leaf_node.Entries["/MediaBox"] + "\n";
            }

            if (pages.page_leaf_node != null && pages.page_leaf_node.Entries.ContainsKey("/Contents") && pages.page_leaf_node.Entries["/Contents"] != null)
            {
                text += "Contents = " + pages.page_leaf_node.Entries["/Contents"] + "\n";
            }

            if (pages.page_leaf_node != null && pages.page_leaf_node.Entries.ContainsKey("/Rotate") && pages.page_leaf_node.Entries["/Rotate"] != null)
            {
                text += "Rotate = " + pages.page_leaf_node.Entries["/Rotate"] + "\n";
            }

            if (pages.page_leaf_node != null && pages.page_leaf_node.Entries.ContainsKey("/Annots") && pages.page_leaf_node.Entries["/Annots"] != null)
            {
                text += "Annots = " + pages.page_leaf_node.Entries["/Annots"] + "\n";
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
