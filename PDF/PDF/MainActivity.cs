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
using iText.Kernel.Pdf;
using iText.Layout;
using iText.Layout.Element;
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
        private readonly List<long> Positions = new List<long>();
        private readonly List<int> Revisions = new List<int>();

        public long GetObjectPosition(int ObjectIndex) => Positions[ObjectIndex];

        public int GetObjectRevision(int ObjectIndex) => Revisions[ObjectIndex];
    }

    public class PDFTrailer
    {
        private PDFCrossReferences CrossReferences;

        private int Size;
        private int RootIndex;
        private int InfoIndex;

        private PDFObject Root;
        private PDFObject Info;

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
            //Size = 1; //Get From Stream
            //RootIndex = 0; //Get From Stream
            //InfoIndex = 0; //Get From Stream
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
        private int OutlinesIndex;
        private int PagesIndex;

        public PDFCatalog(MemoryStream PDFStream, PDFCrossReferences References) : base(PDFStream, References)
        {

        }
    }

    public class PDFPages : PDFObject
    {
        public PDFPages(MemoryStream PDFStream, PDFCrossReferences References) : base(PDFStream, References)
        {

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

            
            /*
            string content;
            AssetManager assets = this.Assets;
            using (StreamReader sr = new StreamReader(assets.Open("PDF.pdf")))
            {
                content = sr.ReadToEnd();
                //Console.WriteLine(content);

            }
            //*/




            /*
            Console.WriteLine("================ReadLine==================");
            using (StreamReader sr = new StreamReader(assets.Open("PDF.pdf")))
            {
                while (sr.Peek() >= 0)
                {
                    Console.WriteLine(sr.ReadLine());
                }

            }
            //*/

            /*
            Console.WriteLine("================ReadLine[start]==================");

            AssetManager assets = this.Assets;
            Stream stream = assets.Open("PDF.pdf");
            MemoryStream memory_stream = new MemoryStream();
            stream.CopyTo(memory_stream);
            memory_stream.Seek(38338, SeekOrigin.Begin);   // From 0, %PDF-1.7 \r\n
            //Console.WriteLine(memory_stream.Position);
            for(int i=0;i<10;i++)
            {
                Console.WriteLine((char)memory_stream.ReadByte()); // From 38338 xref \r\n 0 space 39
            }

            read_pdf_line(memory_stream);

            //using (StreamReader sr = new StreamReader(assets.Open("PDF.pdf")))
            //{
            //    sr.BaseStream.Seek(38338, SeekOrigin.Begin);
            //   Console.WriteLine(sr.ReadLine());


            //}

            /*
            string content;
            AssetManager assets = this.Assets;
            using (StreamReader sr = new StreamReader(assets.Open("PDF.pdf")))
            {
                content = sr.ReadToEnd();
                //Console.WriteLine(content);
            }

            Console.WriteLine("================ReadLine==================");
            using (StreamReader sr = new StreamReader(assets.Open("PDF.pdf")))
            {
                while (sr.Peek() >= 0)
                {
                    Console.WriteLine(sr.ReadLine());
                }

            }
            //*/



            SKCanvasView canvasView = FindViewById<SKCanvasView>(Resource.Id.canvasView);
            canvasView.PaintSurface += OnPaintSurface;

            /*
            Dictionary<string, string> pdf_object = new Dictionary<string, string>();

            string tag = "/Type/Catalog/Pages";
            string tag_1 = "/Lang";
            string tag_2 = "/StructTreeRoot";
            string tag_3 = "/MarkInfo";
            string tag_4 = "/Metadata";
            string tag_5 = "/ViewerPreferences";

            pdf_object.Add(tag, objoct_1.Substring(objoct_1.IndexOf(tag) + tag.Length, objoct_1.IndexOf(tag_1) - ( objoct_1.IndexOf(tag) + tag.Length ) )   );
            
            pdf_object.Add(tag_1, objoct_1.Substring(objoct_1.IndexOf(tag_1) + tag_1.Length, objoct_1.IndexOf(tag_2) - (objoct_1.IndexOf(tag_1) + tag_1.Length)));
            
            pdf_object.Add(tag_2, objoct_1.Substring(objoct_1.IndexOf(tag_2) + tag_2.Length, objoct_1.IndexOf(tag_3) - (objoct_1.IndexOf(tag_2) + tag_2.Length)));
            
            pdf_object.Add(tag_3, objoct_1.Substring(objoct_1.IndexOf(tag_3) + tag_3.Length, objoct_1.IndexOf(tag_4) - (objoct_1.IndexOf(tag_3) + tag_3.Length)));
            
            pdf_object.Add(tag_4, objoct_1.Substring(objoct_1.IndexOf(tag_4) + tag_4.Length, objoct_1.IndexOf(tag_5) - (objoct_1.IndexOf(tag_4) + tag_4.Length)));
        
            pdf_object.Add(tag_5, objoct_1.Substring(objoct_1.IndexOf(tag_5) + tag_5.Length, objoct_1.Length - 4 - (objoct_1.IndexOf(tag_5) + tag_5.Length)));




            string objoct_2 = content.Split("endobj")[1];
            string tag_6 = "/Type/Pages/Count";
            string tag_7 = "/Kids";
            pdf_object.Add(tag_6, objoct_2.Substring(objoct_2.IndexOf(tag_6) + tag_6.Length, objoct_2.IndexOf(tag_7) - (objoct_2.IndexOf(tag_6) + tag_6.Length)));
            
            pdf_object.Add(tag_7, objoct_2.Substring(objoct_2.IndexOf(tag_7) + tag_7.Length, objoct_2.Length - 4 - (objoct_2.IndexOf(tag_7) + tag_7.Length)));


            string objoct_3 = content.Split("endobj")[2];
            string tag_8 =  "/Type/Page/Parent";
            string tag_9 =  "/Resources";
            //string tag_10 = "/Font";
            //string tag_11 = "/ProcSet";
            string tag_12 = "/MediaBox";
            string tag_13 = "/Contents";
            string tag_14 = "/Group";
            string tag_15 = "/Tabs/S/StructParents";

            pdf_object.Add(tag_8, objoct_3.Substring(objoct_3.IndexOf(tag_8) + tag_8.Length, objoct_3.IndexOf(tag_9) - (objoct_3.IndexOf(tag_8) + tag_8.Length)));
            pdf_object.Add(tag_9, objoct_3.Substring(objoct_3.IndexOf(tag_9) + tag_9.Length, objoct_3.IndexOf(tag_12) - (objoct_3.IndexOf(tag_9) + tag_9.Length)));
            //pdf_object.Add(tag_10, objoct_3.Substring(objoct_3.IndexOf(tag_10) + tag_10.Length, objoct_3.IndexOf(tag_11) - (objoct_3.IndexOf(tag_10) + tag_10.Length)));
            //pdf_object.Add(tag_11, objoct_3.Substring(objoct_3.IndexOf(tag_11) + tag_11.Length, objoct_3.IndexOf(tag_12) - (objoct_3.IndexOf(tag_11) + tag_11.Length)));
            pdf_object.Add(tag_12, objoct_3.Substring(objoct_3.IndexOf(tag_12) + tag_12.Length, objoct_3.IndexOf(tag_13) - (objoct_3.IndexOf(tag_12) + tag_12.Length)));
            pdf_object.Add(tag_13, objoct_3.Substring(objoct_3.IndexOf(tag_13) + tag_13.Length, objoct_3.IndexOf(tag_14) - (objoct_3.IndexOf(tag_13) + tag_13.Length)));
            pdf_object.Add(tag_14, objoct_3.Substring(objoct_3.IndexOf(tag_14) + tag_14.Length, objoct_3.IndexOf(tag_15) - (objoct_3.IndexOf(tag_14) + tag_14.Length)));
            pdf_object.Add(tag_15, objoct_3.Substring(objoct_3.IndexOf(tag_15) + tag_15.Length, objoct_3.Length - 4 - (objoct_3.IndexOf(tag_15) + tag_15.Length)));

            string text = "";
            foreach (KeyValuePair<string, string> kvp in pdf_object)
            {
                Console.WriteLine(string.Format("Key = {0}, Value = {1}", kvp.Key, kvp.Value));
                text += string.Format("Key = {0}, Value = {1}" + "\r\n", kvp.Key, kvp.Value);
            }

            AppCompatTextView text_view = FindViewById<AppCompatTextView>(Resource.Id.text_view);
            text_view.SetText(text.ToCharArray(),0,text.Length);
            

            /*
            Console.WriteLine(content.IndexOf("1 0 obj"));
            Console.WriteLine(content.IndexOf("2 0 obj"));
            Console.WriteLine(content.IndexOf("3 0 obj"));
            Console.WriteLine(content.IndexOf("4 0 obj"));
            Console.WriteLine(content.IndexOf("5 0 obj"));
            Console.WriteLine(content.IndexOf("6 0 obj"));
            Console.WriteLine(content.IndexOf("7 0 obj"));
            Console.WriteLine(content.IndexOf("8 0 obj"));
            Console.WriteLine(content.IndexOf("9 0 obj"));
            Console.WriteLine(content.IndexOf("10 0 obj"));
            Console.WriteLine(content.IndexOf("11 0 obj"));
            Console.WriteLine(content.IndexOf("12 0 obj"));
            Console.WriteLine(content.IndexOf("13 0 obj"));
            Console.WriteLine(content.IndexOf("14 0 obj"));
            Console.WriteLine(content.IndexOf("15 0 obj"));
            Console.WriteLine(content.IndexOf("16 0 obj"));
            Console.WriteLine(content.IndexOf("17 0 obj"));
            Console.WriteLine(content.IndexOf("18 0 obj"));
            Console.WriteLine(content.IndexOf("19 0 obj"));
            Console.WriteLine(content.IndexOf("20 0 obj"));
            Console.WriteLine(content.IndexOf("21 0 obj"));
            Console.WriteLine(content.IndexOf("22 0 obj"));
            Console.WriteLine(content.IndexOf("23 0 obj"));
            Console.WriteLine(content.IndexOf("24 0 obj"));
            Console.WriteLine(content.IndexOf("25 0 obj"));
            Console.WriteLine(content.IndexOf("26 0 obj"));
            Console.WriteLine(content.IndexOf("27 0 obj"));
            Console.WriteLine(content.IndexOf("28 0 obj"));
            Console.WriteLine(content.IndexOf("29 0 obj"));
            Console.WriteLine(content.IndexOf("30 0 obj"));
            Console.WriteLine(content.IndexOf("31 0 obj"));
            Console.WriteLine(content.IndexOf("32 0 obj"));
            Console.WriteLine(content.IndexOf("33 0 obj"));
            Console.WriteLine(content.IndexOf("34 0 obj"));
            Console.WriteLine(content.IndexOf("35 0 obj"));
            Console.WriteLine(content.IndexOf("36 0 obj"));
            Console.WriteLine(content.IndexOf("37 0 obj"));
            Console.WriteLine(content.IndexOf("38 0 obj"));
            Console.WriteLine(content.IndexOf("39 0 obj"));
            //*/
            /*
            memory_stream.Position = 0;
            using (PdfReader pdfReader = new PdfReader(memory_stream))
            {
                
                var pdfDocument = new iText.Kernel.Pdf.PdfDocument(pdfReader);
                var contents = iText.Kernel.Pdf.Canvas.Parser.PdfTextExtractor.GetTextFromPage(pdfDocument.GetFirstPage());
                
                Console.WriteLine("================PdfWriter==================");
                Console.WriteLine(contents);
                Console.WriteLine(pdfDocument.GetPage(1).GetResources().GetResourceNames().Count);
                
            }


            //var pdfReader = new iText.Kernel.Pdf.PdfReader(baos);




            //Console.WriteLine("================PdfWriter==================");
            //Console.WriteLine(d.GetDefaultProperty<Text>(1));

            //d.Add(new Paragraph("Hello world!"));

            //d.Close();
            //*/


            /*
            // Must have write permissions to the path folder
            PdfWriter writer = new PdfWriter(assets.Open("PDF.pdf"));
            iText.Kernel.Pdf.PdfDocument pdf = new iText.Kernel.Pdf.PdfDocument(writer);
            Document document = new Document(pdf);

            Console.WriteLine("================PdfWriter==================");
            Console.WriteLine(document.GetPdfDocument().GetNumberOfPages());

            //Paragraph header = new Paragraph("HEADER")
            //   .SetTextAlignment((iText.Layout.Properties.TextAlignment?)TextAlignment.Center)
            //   .SetFontSize(20);

            //document.Add(header);
            document.Close();
            //*/

            /*
            using (var i = this.Assets.Open("PDF.pdf"))
            using (var o = this.OpenFileOutput("_sample.pdf", FileCreationMode.Private)) i.CopyTo(o);
            var f = this.GetFileStreamPath("_sample.pdf");

            ParcelFileDescriptor fd = ParcelFileDescriptor.Open(f, ParcelFileMode.ReadOnly);
            PdfRenderer renderer = new PdfRenderer(fd);
            Bitmap bitmap = Bitmap.CreateBitmap(210, 500, Bitmap.Config.Argb4444);
            PdfRenderer.Page page = renderer.OpenPage(0);
            page.Render(bitmap, null, null, Android.Graphics.Pdf.PdfRenderMode.ForDisplay);


            AppCompatImageView image_view = FindViewById<AppCompatImageView>(Resource.Id.imageView);


            image_view.SetImageBitmap(bitmap);
            //*/


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



            //tr.BaseStream.Position = 4940559;
            //tr.DiscardBufferedData();
            //tr.BaseStream.Seek(4940559,SeekOrigin.Begin);

            //Console.WriteLine("=================tr.ReadLine()=======================");
            //Console.WriteLine(tr.ReadLine());


            // the code that you want to measure comes here

            //string content = tr.ReadToEnd();

            //stream.Position = 0;

            //byte[] streamBytes = new byte[28113840];
            //stream.Read(streamBytes, 0, (int)stream.Length);
            //string stringOfStream = Encoding.UTF32.GetString(streamBytes);
            //if (stringOfStream.Contains("MSTND"))
            //{
            //}

            watch = System.Diagnostics.Stopwatch.StartNew();

            MemoryStream memory_stream = new MemoryStream();
            stream.CopyTo(memory_stream);
            string content = Encoding.ASCII.GetString(memory_stream.ToArray());

            watch.Stop();
            elapsedMs = watch.ElapsedMilliseconds;
            Console.WriteLine("==========ElapsedMilliseconds[GetString]=====");
            Console.WriteLine(elapsedMs);
            Console.WriteLine(memory_stream.CanSeek);
            //memory_stream.read
            


            //stream_text.Seek(4940559,SeekOrigin.Begin);  //not support 
            //stream_text.Position = 4940559;              //not support 


            //TextReader stream_reader = new StreamReader(assets.Open("sample_2.pdf"));

            //content = stream_reader.ReadToEnd();

            //Console.WriteLine("==========content[2].Length========");
            //Console.WriteLine(content.Length);


            /*
            MemoryStream memory_stream = new MemoryStream();
            stream.CopyTo(memory_stream);

            memory_stream.Position = 0;

            byte[] buffer = new byte[5000];

            memory_stream.Position = memory_stream.Length -5;
            memory_stream.Read(buffer, 0, 5);
            Console.WriteLine((int)memory_stream.Length);

            Console.WriteLine((char)buffer[0]);
            Console.WriteLine((char)buffer[1]);
            Console.WriteLine((char)buffer[2]);
            Console.WriteLine((char)buffer[3]);
            Console.WriteLine((char)buffer[4]);



            //memory_stream.Position = 3025;
            //Console.WriteLine((char)memory_stream.ReadByte());

            string content = "";
            //Console.WriteLine(memory_stream.Position);
            for (int i = 0; i < memory_stream.Length; i++)
            {
                //content += (char)memory_stream.ReadByte();
                // From 38338 xref \r\n 0 space 39
            }
            //*/



            string trailer_string = read_trailer(content);

            string startxref = read_int(trailer_string, "startxref");

            //Console.WriteLine("============startxref=============");
            //Console.WriteLine(startxref);
            //Console.WriteLine(read_length(content, int.Parse(startxref)));

            watch = System.Diagnostics.Stopwatch.StartNew();
            Dictionary<string, string> xref = read_xref( content, int.Parse(startxref), read_length(content, int.Parse(startxref) ) );

            watch.Stop();
            elapsedMs = watch.ElapsedMilliseconds;

            Console.WriteLine("==========ElapsedMilliseconds[read_xref]=====");
            Console.WriteLine(elapsedMs);

            Pages complete_pages =  make_pages(content, xref,"2");

            //foreach (KeyValuePair<string, string> kvp in xref)
            //{
            //    Console.WriteLine(string.Format("Key = {0}, Value = {1}", kvp.Key, kvp.Value));
            //}



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
