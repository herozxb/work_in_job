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
            int entry_position = index + 6; // +6 jump the "xref/r/n" length
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

                if (content[entry_position] == '\r' || content[entry_position] == ' ' || content[entry_position] == '/')
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


            stream = assets.Open("sample_2.pdf");
            watch = System.Diagnostics.Stopwatch.StartNew();


            PDFCrossReferences pdf_cross_reference = new PDFCrossReferences(stream);

            PDFTrailer pdf_trailer_object = new PDFTrailer(memory_stream, pdf_cross_reference);

            PDFPages pdf_pages = ((PDFCatalog)(pdf_trailer_object.Root)).Pages;

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
