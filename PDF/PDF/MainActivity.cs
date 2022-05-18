using System;
using Android.App;
using Android.OS;
using Android.Runtime;
using Android.Views;
using AndroidX.AppCompat.Widget;
using AndroidX.AppCompat.App;
using Google.Android.Material.FloatingActionButton;
using Google.Android.Material.Snackbar;
using Android.Content.Res;
using System.IO;
using Android.Graphics.Pdf;
using System.Collections.Generic;
using Android.Graphics;
using Android.Content;
using AndroidX.RecyclerView.Widget;
using SkiaSharp;
using SkiaSharp.Views.Android;
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
        public List<Pages> pages_children_list = new List<Pages>();
        public string context = new string("");
    }

    public class Page
    {
        public Dictionary<string, string> Entries = new Dictionary<string, string>();
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

        public List<long> make_position( List<string> xref )
        {
            List<long> position = new List<long>();
            for (int i = 0; i < xref.Count; i++)
            {
                position.Add(long.Parse(xref[i].Split(" ")[0]));
            }

            return position;
        }

        public List<int> make_revision( List<string> xref)
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

                if (content[entry_position] == '\r' || content[entry_position] == ' ' || content[entry_position] == '/')
                {
                    break;
                }
            }

            return result;
        }

        public string read_length(string content, int index)
        {
            string result = "";
            int entry_position = index + 6; // +6 jump the xref /r/n length
            while (true)
            {
                if (content[entry_position] == '\r')
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
                if (content[entry_position] == '\r')
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
            result = content.Split("trailer")[1];// time = 0.127 
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

                if (content[entry_position] == '\r' || content[entry_position] == ' ' || content[entry_position] == '/')
                {
                    break;
                }
            }

            return result;
        }


        public PagesTreeNode make_pages(string content, List<string> xref, int object_index)
        {

            PagesTreeNode pages_tree_node = new PagesTreeNode();


            int line_number = int.Parse(xref[object_index].Split(" ")[0]);

            string pages_object = this.read_obj(content, line_number);

            string type = this.read_string(pages_object, "/Type");
            string count = this.read_int(pages_object, "/Count");
            string Kids = this.read_array(pages_object, "/Kids");

            Console.WriteLine("================[Pages][start]=====================");
            Console.WriteLine(type);
            Console.WriteLine(count);
            Console.WriteLine(Kids);
            Console.WriteLine("================[Pages][end]=====================");






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
                    //Console.WriteLine("================Kids[start]=====================");
                    //Console.WriteLine(kids_index);
                    //Console.WriteLine("================Kids[end]=====================");

                    if (kids_index.Replace(" ", "").Length > 0)
                    {
                        pages_tree_node.pages_children_list.Add(make_pages(content, xref, int.Parse(kids_index)));
                    }

                }
            }
            else
            {
                Console.WriteLine("================Kids[Stop]=====================");
                PagesTreeNode pages_end = new PagesTreeNode();
                pages_end.Type = "Page";
                pages_tree_node.pages_children_list.Add(pages_end);
            }
            return pages_tree_node;
        }



        public string clean_front_empty_space(string content)
        {
            while(content[0] == ' ')
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
            Root = PDFObject.Create(PDFStream, CrossReferences);

            PDFStream.Position = CrossReferences.GetObjectPosition(InfoIndex);
            Info = PDFObject.Create(PDFStream, CrossReferences);
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

        public static PDFObject Create(MemoryStream PDFStream, PDFCrossReferences References)
        {
            //Get Type;
            string Type = "";
            switch (Type)
            {
                case "Catalog":
                    return new PDFCatalog(PDFStream, References);
                case "Pages":
                    return new PDFPages(PDFStream, References);
                case "Page":
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


            stream = assets.Open("sample_2.pdf");
            watch = System.Diagnostics.Stopwatch.StartNew();


            PDFCrossReferences pdf_cross_reference = new PDFCrossReferences(stream);
            pdf_cross_reference.make_position();

            PDFTrailer pdf_trailer_object = new PDFTrailer(memory_stream, pdf_cross_reference);

            Console.WriteLine(pdf_trailer_object.Size);
            Console.WriteLine(pdf_trailer_object.RootIndex);
            Console.WriteLine(pdf_trailer_object.InfoIndex);

            PDFCatalog pdf_catalog = new PDFCatalog(memory_stream, pdf_cross_reference);

            Console.WriteLine(pdf_catalog.Type);
            Console.WriteLine(pdf_catalog.OutlinesIndex);
            Console.WriteLine(pdf_catalog.PagesIndex);

            PDFPages pdf_pages = new PDFPages(memory_stream, pdf_cross_reference);

            watch.Stop();
            elapsedMs = watch.ElapsedMilliseconds;
            Console.WriteLine("==========ElapsedMilliseconds[GetString]=====");
            Console.WriteLine(elapsedMs);
            Console.WriteLine(memory_stream.CanSeek);
            Console.WriteLine(pdf_cross_reference.GetObjectPosition(0));

            string trailer_string = read_trailer(content);

            string startxref = read_int(trailer_string, "startxref");

            watch = System.Diagnostics.Stopwatch.StartNew();
            Dictionary<string, string> xref = read_xref( content, int.Parse(startxref), read_length(content, int.Parse(startxref) ) );

            watch.Stop();
            elapsedMs = watch.ElapsedMilliseconds;

            Console.WriteLine("==========ElapsedMilliseconds[read_xref]=====");
            Console.WriteLine(elapsedMs);

            Pages complete_pages =  make_pages(content, xref,"2");


            trailer pdf_trailer = new trailer();
            pdf_trailer.Entries.Add("/Size", read_int(trailer_string, "/Size"));
            pdf_trailer.Entries.Add("/Root", read_obj_index(trailer_string, "/Root"));
            pdf_trailer.Entries.Add("/Info", read_obj_index(trailer_string, "/Info"));

            int line_number = int.Parse(xref["1"].Split(" ")[0]);
            string document_catalog_object = read_obj(content, line_number);
            document_catalog document_catalog = new document_catalog();
            document_catalog.Entries.Add("/Type", read_string(document_catalog_object, "/Type"));
            document_catalog.Entries.Add("/Outlines", read_obj_index(document_catalog_object, "/Outlines"));
            document_catalog.Entries.Add("/Pages", read_obj_index(document_catalog_object, "/Pages"));

            
            Pages pages = new Pages();
            line_number = int.Parse(xref["2"].Split(" ")[0]);
            string pages_object = read_obj(content, line_number);
            pages.Entries.Add("/Type", read_string(pages_object, "/Type"));
            pages.Entries.Add("/Count", read_int(pages_object, "/Count"));
            pages.Entries.Add("/Kids", read_array(pages_object, "/Kids"));



            line_number = int.Parse(xref["3353"].Split(" ")[0]);
            string pages_object_2 = read_obj(content, line_number);
            string object_type = read_string(pages_object_2, "/Type");

            Pages pages_2 = new Pages();
            Page page = new Page();
            if (object_type == "/Pages")
            {
                pages_2.Entries.Add("/Type", read_string(pages_object_2, "/Type"));
                pages_2.Entries.Add("/Count", read_int(pages_object_2, "/Count"));
                pages_2.Entries.Add("/Kids", read_array(pages_object_2, "/Kids"));

            }
            else if (object_type == "/Page")
            {

                page.Entries.Add("/Type", read_string(pages_object_2, "/Type"));

                page.Entries.Add("/Parent", read_obj_index(pages_object_2, "/Parent"));
                page.Entries.Add("/MediaBox", read_array(pages_object_2, "/MediaBox"));
                page.Entries.Add("/Contents", read_obj_index(pages_object_2, "/Contents"));

            }


            line_number = int.Parse(xref["4459"].Split(" ")[0]);
            string pages_object_3 = read_obj(content, line_number);
            string object_type_2 = read_string(pages_object_3, "/Type");

            Pages pages_3 = new Pages();
            Page page_2 = new Page();
            if (object_type == "/Pages")
            {


                pages_3.Entries.Add("/Type", read_string(pages_object_3, "/Type"));
                pages_3.Entries.Add("/Count", read_int(pages_object_3, "/Count"));
                pages_3.Entries.Add("/Kids", read_array(pages_object_3, "/Kids"));

            }
            else if (object_type == "/Page")
            {

                page_2.Entries.Add("/Type", read_string(pages_object_3, "/Type"));

                page_2.Entries.Add("/Parent", read_obj_index(pages_object_3, "/Parent"));
                page_2.Entries.Add("/MediaBox", read_array(pages_object_3, "/MediaBox"));
                page_2.Entries.Add("/Contents", read_obj_index(pages_object_3, "/Contents"));

            }


            /*
            line_number = int.Parse(xref["4459"].Split(" ")[0]);
            string page_object = read_obj(content, line_number);
            Page page_2 = new Page();
            page_2.Entries.Add("/Type", read_string(page_object, "/Type"));
            page_2.Entries.Add("/Parent", read_obj_index(page_object, "/Parent"));
            page_2.Entries.Add("/MediaBox", read_array(page_object, "/MediaBox"));
            page_2.Entries.Add("/Contents", read_obj_index(page_object, "/Contents"));
            //*/

            
            string text = "";

            visit_tree_node(complete_pages, ref text);

            /*
            text = text + "[trailer]\r\n";
            foreach (KeyValuePair<string, string> kvp in pdf_trailer.Entries)
            {
                Console.WriteLine(string.Format("Key = {0}, Value = {1}", kvp.Key, kvp.Value));
                text += string.Format("Key = {0}, Value = {1}" + "\r\n", kvp.Key, kvp.Value);
            }

            text = text + "[document_catalog]\r\n";
            foreach (KeyValuePair<string, string> kvp in document_catalog.Entries)
            {
                Console.WriteLine(string.Format("Key = {0}, Value = {1}", kvp.Key, kvp.Value));
                text += string.Format("Key = {0}, Value = {1}" + "\r\n", kvp.Key, kvp.Value);
            }

        
            text = text + "[Pages]\r\n";
            foreach (KeyValuePair<string, string> kvp in pages.Entries)
            {
                Console.WriteLine(string.Format("Key = {0}, Value = {1}", kvp.Key, kvp.Value));
                text += string.Format("Key = {0}, Value = {1}" + "\r\n", kvp.Key, kvp.Value);
            }

            
            text = text + "[Page]\r\n";
            foreach (KeyValuePair<string, string> kvp in pages_2.Entries)
            {
                Console.WriteLine(string.Format("Key = {0}, Value = {1}", kvp.Key, kvp.Value));
                text += string.Format("Key = {0}, Value = {1}" + "\r\n", kvp.Key, kvp.Value);
            }

            text = text + "[Page 2]\r\n";
            foreach (KeyValuePair<string, string> kvp in pages_3.Entries)
            {
                Console.WriteLine(string.Format("Key = {0}, Value = {1}", kvp.Key, kvp.Value));
                text += string.Format("Key = {0}, Value = {1}" + "\r\n", kvp.Key, kvp.Value);
            }

            //*/


            AppCompatTextView text_view = FindViewById<AppCompatTextView>(Resource.Id.text_view);
            text_view.SetText(text.ToCharArray(), 0, text.Length);

        }

        public string read_int(string content, string label)
        {
            string result = "";
            int entry_position = content.IndexOf(label) + label.Length ;
            while (true)
            {
                result = result + content[entry_position];
                entry_position++;

                if (content[entry_position] == '\r' || content[entry_position] == ' ' || content[entry_position] == '/')
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
                if (content[entry_position] == '\r')
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
                if (content[entry_position] == '\r')
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

            //Console.WriteLine("content[entry_position]");
            //Console.WriteLine(entry_position);
            //Console.WriteLine(content[entry_position]);

            //Console.WriteLine(read_xref_line(content, entry_position));

            for ( int i = 0; i< length; i++ )
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
                if (content[entry_position] == 'e' && content[entry_position+1] == 'n' && content[entry_position+2] == 'd' && content[entry_position+3] == 'o' && content[entry_position+4] == 'b' && content[entry_position+5] == 'j')
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
            int entry_position = content.IndexOf(label) + label.Length;
            while (content[entry_position] != ']')
            {
                result = result + content[entry_position];
                entry_position++;
            }

            return result+']';
        }

        public string read_string(string content, string label)
        {
            string result = "";
            int entry_position = content.IndexOf(label) + label.Length;

            while (true)
            {
                result = result + content[entry_position];
                entry_position++;

                if (content[entry_position] == '\r' || content[entry_position] == ' ' || content[entry_position] == '/')
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
            string result ="";
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

            return result ;
        }

        public string read_text_position(string content)
        {
            string result = "";
            int entry_position = content.IndexOf("Td")-1;
            while (content[entry_position] != '\n')
            {
                result = content[entry_position]+result;
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

        public Pages make_pages(string content, Dictionary<string, string> xref, string object_index)
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
            Console.WriteLine(pages_object);
            Console.WriteLine(Kids);
            Console.WriteLine("================[Pages][end]=====================");

            if (type.Contains("/Pages") && pages_object.Contains("/Kids"))
            {
                for (int i = 0; i < Kids.Split("R").Length; i++)
                {
                    string kids_index = Kids.Split("R")[i];
                    //Console.WriteLine("================Kids[start]=====================");
                    //Console.WriteLine(kids_index);
                    //Console.WriteLine("================Kids[end]=====================");
                    if(kids_index.Replace(" ","").Length>0)
                    {
                        pages.pages_children_list.Add(make_pages(content, xref, kids_index));
                    }
                }
            }
            else
            {
                Console.WriteLine("================Kids[Stop]=====================");
                Pages pages_end = new Pages();
                pages_end.Entries.Add("/Kids", "None");
                pages.pages_children_list.Add(pages_end);
            }
            return pages;
        }


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

            if (pages.context != null)
            {
                if (pages.context.Length > 10)
                {
                    text += "ID = " + pages.context.Substring(0, 10) + "\n";
                }
                else if (pages.context.Length <= 10 && pages.context.Length >=7 )
                {
                    text += "ID = " + pages.context.Substring(0, 6) + "\n";
                }
                
            }

            if (pages.Entries.ContainsKey("/Kids") && pages.Entries.ContainsKey("/Count") && pages.Entries["/Kids"] != null && pages.Entries["/Count"].Replace(" ", "") != "0")
            {
                text += "Kids = " + pages.Entries["/Kids"] + "\n";
            }


            for (int i = 0; i < pages.pages_children_list.Count; i++)
            {
                if (pages.Entries.ContainsKey("/Type") && pages.Entries["/Type"].Contains("Page") )
                {
                    text += "====================[Page]=====================\n";
                }

                if (pages.Entries["/Kids"] != "None")
                {
                    visit_tree_node(pages.pages_children_list[i], ref text);
                }
            }

            return text;
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
            View view = (View) sender;
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
