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




            //stream = assets.Open("sample_2.pdf");
            //PDFCrossReferences pdf_cross_reference = new PDFCrossReferences(stream);

            //PDFTrailer pdf_trailer_object = new PDFTrailer(memory_stream, pdf_cross_reference);

            //PDFPages pdf_pages = ((PDFCatalog)(pdf_trailer_object.Root)).Pages;


            string trailer_string = read_trailer(content);

            string startxref = read_int(trailer_string, "startxref");


            watch = System.Diagnostics.Stopwatch.StartNew();
            Dictionary<string, string> xref = read_xref(content, int.Parse(startxref), read_length(content, int.Parse(startxref)));

            watch.Stop();
            elapsedMs = watch.ElapsedMilliseconds;

            Console.WriteLine("==========ElapsedMilliseconds[read_xref]=====");
            Console.WriteLine(elapsedMs);


            int line_number = int.Parse(xref["10230"].Split(" ")[0]);

            string pages_object = read_obj(content, line_number);

            int stream_start = line_number + search_position_from_content(pages_object, "stream") + "stream".Length+2;//2 is for "/r/n"
            int stream_end = line_number + search_position_from_content(pages_object, "endstream")-2; //2 is for "/r/n"

            Console.WriteLine("=================stream_start==================");
            for (int i = 0; i < 10; i++)
            {
                Console.Write(content[stream_start + i]);
            }
            Console.WriteLine("===========");
            for (int i = 0; i < 10; i++)
            {
                Console.Write(content[stream_end + i]);
            }

            stream = assets.Open("sample_2.pdf");
            stream.CopyTo(memory_stream);
            memory_stream.Position = stream_start;

            byte[] byte_stream = new byte[10000];

            int length_stream = stream_end - stream_start;
            Console.WriteLine("=============length_stream===============");//37806870454946541337636363631310493248321119810613606047779711410773110102111606047779711410710110032102971081151016262477897109101115325457525232483282477911711610810511010111532494849574832483282477710111697100971169732515351324832824765991141117011111410932515352324832824780971031011153250324832824783116114117991168411410110182111111116325356324832824784121112101476797116971081111034776971101034010111045858341477911711611211711673110116101110116115916060477311010211140671141019711611111458327280323232323277971101171029799116117114101114587369673232323277111100101108581158271664147834771848395806870654947791171161121171166711111010010511610511111073100101110116105102105101114401158271664147681011151167911711611211711680114111102105108101325153503248328247841211121014779117116112117116731101161011101164782101103105115116114121789710910140104116116112584747119119119469911110811111446111114103416262936262131011101001119810613545752523248321119810613606047681011151161153254575253324832826262131011101001119810613494849574832483211198106136060477010511411511632494849505232483282476711111711011632545447769711511632494849505232483282478412111210147791171161081051101011156262131011101001119810613515351324832111981061360604783117981161211121014788777647761011101031161043251575052478412111210147771011169710097116976262115116114101971091310606312011297991071011163298101103105110613463636334321051006134875377487711267101104105721221141018312278849912210799571003463621060120581201091121091011169732120109108110115581206134971001119810158110115581091011169747343212058120109112116107613465100111981013288778032671111141013252464845995149543252524650535157504944328311711032799911632484932504848543249555849525851573462103232326011410010258826870321201091081101155811410010261341041161161125847471191191194611951461111141034749575757474850475050451141001024511512111011697120451101153534621032323232323260114100102586810111599114105112116105111110321141001025897981111171166134341032323232323232323232323212010910811011558112100102613410411611611258474711011546971001119810146991111094711210010247494651473462================[Pages][start]=====================
            //13107213714086911111959161262239522571479893143239126132801623211218410031144215377733168105123146150352546151991111185521181701182272920722915524919822762239155179197821392456510425419135245771152461069223142058922310711621911061782021292471211714121019813723218820270244187701381822552091041335824597631984139186874398418728212381292503074014130113177224624154254641182021389617213814213017222818722228221230622321597911191352551051912441112083790122212011181401314673222572291631729403158182872426115495144179182179114211118322471631591910720453132982913824524214244541001877039219171109891181341572221472361182442384122121577241281731552612392720865249369214202102225188225122431516423655205852431881591441781051998542431194105479220420219714514214176240206191167185641278714610224470823763181160177301571477512924592182121162176110147196228146188341734517017547909150362411092251302436118119731823624921621822613177121181711111991841079492181246922912319219223624314643103161161381817415582541592371801081561591011061561372112542636472544065861151788321115114911615713651122210717614922018240362247136171739121222187881141391651365118229178157231116722710318441182155208821972012410457159831871211232436411718925195128138144782035110712724610375502141518524441892301032264222916121018534158554021229223025473932041033100163821638384433552271032271292441422311701602101931581931549720216784857835791391926120148175175249149143164102022092208111593962519234119224228235549669177139144215462432419257803616823138120232243163244261571051697424722623525419221523238136210234962002019119120921424749206225104185223321823991612133087702544435923457402132002023117987672462431959613110338223919210821414619221821789404247381324138181451765258882011903722412421295424512670887022221010595227199323823824816067572132473523213614723611630107302414324925552311222418114117821815710618720223021639119165782233224111433266224572191311021371921431752482511075813014125118710223322416493662251701991571921882401102251471866687154235248635489145232251150932411711681142721215816365592421081456725181672514241171031911602442977237481731612072113857175151251985792238203197191116185393238255940119818223148912925515216241245188107128110118451741551012431612494519206981861991310101================[Pages][start]=====================

            //x½U]oÓ0}ÿp¤:vü-MÖ"
            //Ý	¡j¡d¥M ~ÿë¤Ñ6%ÝCSÈ¾=>çøæ8ÏöÕö&[WprUU¶þU<,«ªÜ] Çé¯oy¼È6Û"«¶e/~ª|é¼,«|z
            //Ãñâé+ØüïaÀ¨\`þG¥Õ¦rTXïÂ çUÆ%K¸ÀÙÝÕµÒCJSp*5ìó0øð0¦¸èçIHoÂ×j¢V·ËSTb°iL-À`+Ñ5¤oÂ`DµlÂ-5T«o

            memory_stream.Read(byte_stream, 0, length_stream);

            Console.WriteLine(length_stream);
            for (int i = 0; i < length_stream ; i++)
            {
                //byte_stream[i] = (byte)memory_stream.ReadByte();
                //Console.Write((char)byte_stream[i]);
            }

            byte[] cutinput = new byte[byte_stream.Length - 2];
            Array.Copy(byte_stream, 2, cutinput, 0, cutinput.Length);

            MemoryStream stream_output = new MemoryStream();

            using (MemoryStream compressStream = new MemoryStream(cutinput))
            using (DeflateStream decompressor = new DeflateStream(compressStream, CompressionMode.Decompress))
                decompressor.CopyTo(stream_output);

            string output_result = Encoding.Default.GetString(stream_output.ToArray());

            Console.WriteLine("==========output_result[start]===========");
            Console.WriteLine(output_result);
            Console.WriteLine("==========output_result[end]===========");

            /*
            using (MemoryStream decompressedStream = new MemoryStream(cutinput))
            {
                using (DeflateStream deflateStream = new DeflateStream(decompressedStream, CompressionMode.Decompress))
                {
                    deflateStream.CopyTo(decompressedStream);
                    byte[] result = decompressedStream.ToArray();
                    Console.WriteLine("============decompressedStream===============");
                    for (int i = 0; i < result.Length; i++)
                    {
                        Console.Write(result[i]);
                    }
                }
            }
            //*/
            /*

            int line_number = int.Parse(xref["6942"].Split(" ")[0]);

            string pages_object = read_obj(content, line_number);
            string stream_object = read_stream(pages_object, 0);

            Console.WriteLine(stream_object);

            //byte[] byte_array = Decompress(System.Text.Encoding.UTF8.GetBytes(stream_object));
            //byte[] byte_array = System.Text.Encoding.UTF8.GetBytes(stream_object);


            byte[] byte_array = System.Text.Encoding.UTF8.GetBytes(stream_object);
            Console.WriteLine("==========DeflateStream[start]===========");
            Console.WriteLine(byte_array.Length);

            byte[] cutinput = new byte[byte_array.Length - 2];
            Array.Copy(byte_array, 2, cutinput, 0, cutinput.Length);

            var stream_output = new MemoryStream();

            using (var compressStream = new MemoryStream(cutinput))
            using (var decompressor = new DeflateStream(compressStream, CompressionMode.Decompress))
            {
                decompressor.CopyTo(stream_output);
                Console.WriteLine("==========DeflateStream[start]===========");
                Console.WriteLine(stream_output.Length);
                Console.WriteLine("==========DeflateStream[end]===========");
                for (int i = 0; i < stream_output.Length; i++)
                {
                    Console.Write(stream_output.ReadByte());

                }
            }
                    




            using (MemoryStream decompressedStream = new MemoryStream())
            {
                using (Stream compressStream = GenerateStreamFromString(stream_object))
                {
                    using (MemoryStream memory_stream_inside = new MemoryStream())
                    {
                        compressStream.CopyTo(memory_stream_inside);
                        Console.WriteLine("==========[start][1]==========");
                        Console.WriteLine(memory_stream_inside.Length);
                        Console.WriteLine("==========[end][1]==========");

                        using (DeflateStream deflateStream = new DeflateStream(memory_stream_inside, CompressionMode.Decompress))
                        {
                            deflateStream.CopyTo(decompressedStream);
                            Console.WriteLine("==========[start]==========");
                            Console.WriteLine(deflateStream.ReadByte());
                            for (int i = 0; i < decompressedStream.ToArray().Length; i++)
                            {
                                Console.Write(decompressedStream.ToArray()[i]);
                                Console.Write(decompressedStream.Length);
                            }

                            Console.WriteLine("==========[end]==========");
                        }
                    }
                }


            }

            //*/

            /*
            Pages complete_pages = make_pages(content, xref, "2");

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
                    if (kids_index.Replace(" ", "").Length > 0)
                    {
                        pages.pages_tree_children_node_list.Add(make_pages(content, xref, kids_index));
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
                leaf_node_page.Entries.Add("/Contents", read_obj_index(pages_object, "/Contents"));
                Console.WriteLine(leaf_node_page.Entries["/Parent"]);

                pages_end.page_leaf_node = leaf_node_page;
                pages.pages_tree_children_node_list.Add(pages_end);
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
