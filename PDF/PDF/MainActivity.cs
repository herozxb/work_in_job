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

namespace PDF
{
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

            
            string content;
            AssetManager assets = this.Assets;
            using (StreamReader sr = new StreamReader(assets.Open("PDF.pdf")))
            {
                content = sr.ReadToEnd();
                //Console.WriteLine(content);

            }




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


            
            //AssetManager assets = this.Assets;
            Stream stream = assets.Open("PDF.pdf");
            MemoryStream memory_stream = new MemoryStream();
            stream.CopyTo(memory_stream);

            memory_stream.Position = 0;

            string line_result = "";
            //Console.WriteLine(memory_stream.Position);
            for (int i = 0; i < memory_stream.Length; i++)
            {
                line_result += (char)memory_stream.ReadByte();
                // From 38338 xref \r\n 0 space 39
            }


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
