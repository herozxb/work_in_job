﻿using System;
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
using System.Xml;
using SkiaSharp;
using SkiaSharp.Views.Android;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using Android.Content;
using System.IO.Packaging;
using System.Text;
using System.Xml.Linq;

namespace xps_parser
{
    public class XPSMatrix
    {
        public float Determinant { get; }
        public bool HasInverse { get; }
        public static XPSMatrix Identity { get; }
        public bool IsIdentity { get; }
        public float M11 { get; set; }
        public float M12 { get; set; }
        public double M21 { get; set; }
        public double M22 { get; set; }
        public double OffsetX { get; set; }
        public double OffsetY { get; set; }
    }

    public class XPSGeneralTransform
    {
        public XPSGeneralTransform Inverse { get; }
    }

    public class XPSTransform
    {
        public static XPSTransform Identity { get; }
        public XPSGeneralTransform Inverse { get; }
        public XPSMatrix Value { get; }
    }

    public struct XPSColor
    {
        public byte A { get; set; }
        public byte B { get; set; }
        public byte G { get; set; }
        public byte R { get; set; }
        public float ScA { get; set; }
        public float ScB { get; set; }
        public float ScG { get; set; }
        public float ScR { get; set; }
        public string ColorContext { get; }
    }
    public class XPSVisual
    {
        public XPSBrush Fill { get; set; } //System.Windows.Media.Brush, class SolidColorBrush : System.Windows.Media.Brush
        public XPSGeometry Clip { get; set; }
        public double Opacity { get; set; }
        public XPSBrush OpacityMask { get; set; }
        public string RenderTransform { get; set; }
        public string Name { get; set; }
        public string xml_lang { get; set; }
    }

    //XPS classes dependents are SKRect, SKCanvas, SKPath, SKPaint, SKTextBlob, XmlReader, Java.Util.Zip.ZipFile
    public class XPSGlyphs : XPSVisual
    {
        public int BidiLevel { get; set; }
        public string CaretStops { get; set; }
        public string DeviceFontName { get; set; }
        public float FontRenderingEmSize { get; set; }
        public Uri FontUri { get; set; }
        public string Indices { get; set; }
        public bool IsSideways { get; set; }
        public float OriginX { get; set; }
        public float OriginY { get; set; }
        public string UnicodeString { get; set; }

        private SKTextBlob FrameworkTextData;
        private List<float> GlyphLeftList = new List<float>();
        private List<float> GlyphAdvancesRatio = new List<float>();
        private SKRect[] GlyphBounds;

        public XPSGlyphs(XmlReader Source, Java.Util.Zip.ZipFile XpsZipFile)
        {
            Parse(Source, XpsZipFile);
        }

        private void Parse(XmlReader Source, Java.Util.Zip.ZipFile XpsZipFile)
        {
            BidiLevel = int.Parse(Source.GetAttribute("BidiLevel"));
            FontRenderingEmSize = float.Parse(Source.GetAttribute("FontRenderingEmSize"));
            FontUri = new Uri(Source.GetAttribute("FontUri"));
            Indices = Source.GetAttribute("Indices");
            OriginX = float.Parse(Source.GetAttribute("OriginX"));
            OriginY = float.Parse(Source.GetAttribute("OriginY"));
            UnicodeString = Source.GetAttribute("UnicodeString");
            Fill = new XPSSolidColorBrush(new SKColor(uint.Parse(Source.GetAttribute("Fill").Substring(1), System.Globalization.NumberStyles.HexNumber)));
            OnParsed(XpsZipFile);

            //read the object Elements
            while (Source.Read())
            {
                if (Source.NodeType == XmlNodeType.Element || Source.NodeType == XmlNodeType.Text || Source.NodeType == XmlNodeType.EndElement)
                {
                    if (Source.Name == "Glyphs" && Source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + Source.Name + ">");
                        Source.Read();
                        break;
                    }
                }
            }

        }

        private void OnParsed(Java.Util.Zip.ZipFile zip_file)
        {
            Stream FontStream = DeobfuscateXpsFont_from_stream(FontUri.AbsolutePath, zip_file.GetInputStream(zip_file.GetEntry(FontUri.AbsolutePath.Substring(1))));
            SKTypeface TypeFace = SKTypeface.FromStream(FontStream);
            FontStream.Close();

            SKPaint DrawingAttribute = Fill.ToPaint();
            DrawingAttribute.Typeface = TypeFace;
            DrawingAttribute.TextSize = FontRenderingEmSize;

            string strRegex_compare = @"[-+]?\,[0-9]*\.?[0-9]*";
            Regex myRegex_compare = new Regex(strRegex_compare, RegexOptions.None);
            foreach (Match myMatch in myRegex_compare.Matches(Indices))
            {
                if (myMatch.Success)
                {
                    if (myMatch.ToString().Length != 0)
                    {
                        GlyphAdvancesRatio.Add(Convert.ToSingle(myMatch.ToString().Remove(0, 1)));
                    }
                }
            }

            float[] GlyphAdvances = DrawingAttribute.GetGlyphWidths(UnicodeString, out SKRect[] bounds);
            GlyphBounds = bounds;
            float GlyphLeft = OriginX;
            for (int n = 0; n < GlyphAdvances.Length; n++)
            {
                GlyphLeftList.Add(GlyphLeft);
                if (n < GlyphAdvancesRatio.Count)
                {
                    GlyphLeft += FontRenderingEmSize * GlyphAdvancesRatio[n] / 100;
                }
                else
                {
                    GlyphLeft += GlyphAdvances[n];
                }
            }
            SKTextBlobBuilder BlobBuilder = new SKTextBlobBuilder();
            BlobBuilder.AddHorizontalRun(DrawingAttribute.GetGlyphs(UnicodeString), DrawingAttribute.ToFont(), GlyphLeftList.ToArray(), OriginY);
            FrameworkTextData = BlobBuilder.Build();
        }

        public void Draw(SKCanvas canvas)
        {
            canvas.DrawText(FrameworkTextData, 0, 0, Fill.ToPaint());
        }

        public int HitTest(float x, float y)
        {
            for (int i = 0; i < GlyphBounds.Length; i++)
            {
                if (GlyphBounds[i].Contains(x, y))
                {
                    return i;
                }
            }
            return -1;
        }

        private static Stream DeobfuscateXpsFont_from_stream(string input_font_path, Stream image_header_stream)
        {
            string XpsFontFilename = input_font_path.Split("/")[2];
            int length = 512 * 1024;
            byte[] dta = new byte[length];
            image_header_stream.Read(dta, 0, dta.Length);
            {
                string guid = new Guid(XpsFontFilename.Split('.')[0]).ToString("N");
                byte[] guidBytes = new byte[16];
                for (int i = 0; i < guidBytes.Length; i++)
                {
                    guidBytes[i] = Convert.ToByte(guid.Substring(i * 2, 2), 16);
                }

                for (int i = 0; i < 32; i++)
                {
                    int gi = guidBytes.Length - (i % guidBytes.Length) - 1;
                    dta[i] ^= guidBytes[gi];
                }
            }
            Stream stream = new MemoryStream(dta);
            return stream;
        }
    }


    public class XPSPath : XPSVisual
    {
        public string Data { get; set; } //System.Windows.Media.Geometry
        //public string RenderTransform { get; set; } //System.Windows.Media.Geometry
        SKPath path;

        List<XPSPathFigure> xps_path_figure_list = new List<XPSPathFigure>();
        XPSPathFigure xps_path_figure = new XPSPathFigure();
        SKMatrix pre_matrix;
        string Transformr;
        SKMatrix transform_matrix_geometry;

        string Data_signal;
        string RenderTransform_signal;
        string Stroke_signal;
        string PathFigure_signal;

        public XPSPath(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {
            if( source.GetAttribute("Data") != null )
            {
                this.Data_signal = "Data";
                Data = source.GetAttribute("Data");
                path = SKPath.ParseSvgPathData(Data);
            }

            if (source.GetAttribute("RenderTransform") != null )
            {
                this.RenderTransform_signal = "RenderTransform";
                RenderTransform = source.GetAttribute("RenderTransform");
            }

            if (source.GetAttribute("Stroke") != null)
            {
                Stroke_signal = "Stroke";
                XPSSolidColorBrush brush = new XPSSolidColorBrush( new SKColor(uint.Parse(source.GetAttribute("Stroke").Substring(1), System.Globalization.NumberStyles.HexNumber)) );
                brush.InternalPaint.Style = SKPaintStyle.Stroke;
                brush.InternalPaint.StrokeWidth = float.Parse( source.GetAttribute("StrokeThickness") );
                this.Fill = brush;

                source.ReadToDescendant("PathGeometry");

                if (source.GetAttribute("Transform") != null)
                {
                    this.Transformr = source.GetAttribute("Transform");
                    this.transform_matrix_geometry = new SKMatrix(float.Parse(Transformr.Split(",")[0]), float.Parse(Transformr.Split(",")[2]), float.Parse(Transformr.Split(",")[4]), float.Parse(Transformr.Split(",")[1]), float.Parse(Transformr.Split(",")[3]), float.Parse(Transformr.Split(",")[5]), 0, 0, 1);
                }

                while (source.Read())
                {
                    if (source.Name == "PathFigure" && source.GetAttribute("StartPoint") != null)
                    {
                        PathFigure_signal = "PathFigure";
                        XPSPathFigure xps_path_figure = new XPSPathFigure();
                        xps_path_figure.parse_PathFigure(source);
                        xps_path_figure_list.Add(xps_path_figure);

                    }

                    if (source.Name == "PathGeometry" && source.NodeType == XmlNodeType.EndElement)
                    {
                        source.Read();
                        break;
                    }
                }
                
            }
            else
            {

                if (source.ReadToDescendant("Path.Fill"))
                {
                    source.Read();
                    source.Read();

                    if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                    {
                        if (source.Name == "SolidColorBrush")
                        {
                            this.Fill = new XPSSolidColorBrush(source, zip_file);
                        }

                        if (source.Name == "ImageBrush")
                        {
                            this.Fill = new XPSImageBrush(source, zip_file);
                        }

                        if (source.Name == "RadialGradientBrush")
                        {
                            this.Fill = new XPSRadialGradientBrush(source, zip_file);
                        }

                        if (source.Name == "LinearGradientBrush")
                        {
                            this.Fill = new XPSLinearGradientBrush(source, zip_file);
                        }

                        if (source.Name == "VisualBrush")
                        {
                            this.Fill = new XPSVisualBrush(source, zip_file);
                        }
                    }
                }
                else
                {
                    this.Fill = new XPSSolidColorBrush(new SKColor(0, 255, 0));
                }

                while (source.Read())
                {

                    if (source.Name == "PathGeometry" && source.NodeType == XmlNodeType.Element)
                    {
                        if (source.GetAttribute("Transform") != null)
                        {
                            this.Transformr = source.GetAttribute("Transform");
                            this.transform_matrix_geometry = new SKMatrix(float.Parse(Transformr.Split(",")[0]), float.Parse(Transformr.Split(",")[2]), float.Parse(Transformr.Split(",")[4]), float.Parse(Transformr.Split(",")[1]), float.Parse(Transformr.Split(",")[3]), float.Parse(Transformr.Split(",")[5]), 0, 0, 1);
                        }
                    }

                    if (source.Name == "PathFigure" && source.GetAttribute("StartPoint") != null)
                    {
                        PathFigure_signal = "PathFigure";
                        xps_path_figure.parse_PathFigure(source);
                        xps_path_figure.BuildInternalData();

                    }

                    if (source.Name == "Path.Data" && source.NodeType == XmlNodeType.EndElement)
                    {
                        break;
                    }
                    else if (source.Name == "Path" && source.NodeType == XmlNodeType.EndElement)
                    {
                        break;
                    }
                }
            }
        }

        public void Draw(SKCanvas canvas)
        {
            if (this.Fill != null && this.Fill.ToString() == "xps_parser.XPSImageBrush")
            {
                canvas.DrawPath(path, this.Fill.ToPaint());
            }

            if (this.Fill != null && this.Fill.ToString() == "xps_parser.XPSRadialGradientBrush")
            {
                if (RenderTransform_signal == "RenderTransform")
                {
                    this.transform_matrix_geometry = new SKMatrix(float.Parse(RenderTransform.Split(",")[0]), float.Parse(RenderTransform.Split(",")[2]), float.Parse(RenderTransform.Split(",")[4]), float.Parse(RenderTransform.Split(",")[1]), float.Parse(RenderTransform.Split(",")[3]), float.Parse(RenderTransform.Split(",")[5]), 0, 0, 1);
                    this.pre_matrix = canvas.TotalMatrix;
                    SKMatrix post_matrix = SKMatrix.CreateIdentity();
                    SKMatrix.Concat(ref post_matrix, canvas.TotalMatrix, this.transform_matrix_geometry);

                    canvas.SetMatrix(post_matrix);


                    if (Data_signal == "Data")
                    {
                        canvas.DrawPath(path, this.Fill.ToPaint());
                    }

                    if (PathFigure_signal == "PathFigure")
                    {
                        canvas.DrawPath(xps_path_figure.Data, this.Fill.ToPaint());
                    }

                    canvas.SetMatrix(pre_matrix);
                }
                else
                {
                    if (Data_signal == "Data")
                    {
                        canvas.DrawPath(path, this.Fill.ToPaint());
                    }

                    if (PathFigure_signal == "PathFigure")
                    {
                        canvas.DrawPath(xps_path_figure.Data, this.Fill.ToPaint());
                    }

                }

            }


            if (this.Fill != null && this.Fill.ToString() == "xps_parser.XPSLinearGradientBrush")
            {
                if (RenderTransform_signal == "RenderTransform")
                {
                    this.transform_matrix_geometry = new SKMatrix(float.Parse(RenderTransform.Split(",")[0]), float.Parse(RenderTransform.Split(",")[2]), float.Parse(RenderTransform.Split(",")[4]), float.Parse(RenderTransform.Split(",")[1]), float.Parse(RenderTransform.Split(",")[3]), float.Parse(RenderTransform.Split(",")[5]), 0, 0, 1);
                    this.pre_matrix = canvas.TotalMatrix;
                    SKMatrix post_matrix = SKMatrix.CreateIdentity();
                    SKMatrix.Concat(ref post_matrix, canvas.TotalMatrix, this.transform_matrix_geometry);

                    canvas.SetMatrix(post_matrix);
                    canvas.DrawPath(path, this.Fill.ToPaint());
                    canvas.SetMatrix(pre_matrix);
                }
                else
                {
                    canvas.DrawPath(path, this.Fill.ToPaint());
                }
            }

            if (Stroke_signal == "Stroke")
            {
                this.pre_matrix = canvas.TotalMatrix;
                SKMatrix post_matrix = SKMatrix.CreateIdentity();
                SKMatrix.Concat(ref post_matrix, canvas.TotalMatrix, this.transform_matrix_geometry);
                canvas.SetMatrix(post_matrix);

                for (int i = 0; i < xps_path_figure_list.Count; i++)
                {
                    xps_path_figure_list[i].BuildInternalData();
                    canvas.DrawPath(xps_path_figure_list[i].Data, this.Fill.ToPaint());
                }

                canvas.SetMatrix(pre_matrix);
            }

            if (this.Fill != null && this.Fill.ToString() == "xps_parser.XPSVisualBrush")
            {
                canvas.DrawPath(path, this.Fill.ToPaint());
            }
        }
    }

    public class XPSCanvas : XPSVisual
    {
        public List<XPSVisual> xps_visual_children = new List<XPSVisual>(0);
    }

    public class XPSFixedPage : XPSVisual
    {
        public XPSBrush Background { get; set; }
        public SKRect BleedBox { get; set; }
        public SKRect ContentBox { get; set; }
        public object PrintTicket { get; set; }
    }
    public abstract class XPSAnimatable
    {
        public bool HasAnimatedProperties { get; }
    }

    public class XPSBrush : XPSAnimatable
    {
        public SKPaint InternalPaint = new SKPaint();

        public double Opacity { get; set; }
        public XPSTransform RelativeTransform { get; set; } // public abstract class Transform : System.Windows.Media.GeneralTransform
        public XPSTransform Transform { get; set; }         // public abstract class Transform : System.Windows.Media.GeneralTransform

        public SKPaint ToPaint()
        {
            return InternalPaint;
        }
    }

    public class XPSSolidColorBrush : XPSBrush
    {
        public XPSSolidColorBrush(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {
            InternalPaint.Color = new SKColor(uint.Parse(source.GetAttribute("Color").Substring(1), System.Globalization.NumberStyles.HexNumber));
        }

        public XPSSolidColorBrush(SKColor FillColor)
        {
            InternalPaint.Color = FillColor;
        }
    }


    public abstract class XPSTileBrush : XPSBrush
    {
        public enum AlignmentX { Left, Center, Right };
        public enum AlignmentY { Top, Center, Bottom };
        public enum Stretch { None, Fill, Uniform, UniformToFill };
        public enum TileMode { None, FlipX, FlipY, FlipXY, Tile };
        public enum BrushMappingMode { Absolute, RelativeToBoundingBox };
        public SKRect Viewbox { get; set; }
        public BrushMappingMode ViewboxUnits { get; set; }
        public SKRect Viewport { get; set; }
        public BrushMappingMode ViewportUnits { get; set; }
    }



    public class XPSImageBrush : XPSTileBrush
    {
        public string ImageSource { get; set; }

        string text_matrix;

        public SKCanvas canvas;
        public XmlReader source;
        public Java.Util.Zip.ZipFile zip_file;
        public SKPaint ImagePaint;



        public XPSImageBrush(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {
            Parse(source, zip_file);
        }

        private void Parse(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {

            this.ImageSource = source.GetAttribute("ImageSource");

            string[] viewboxContent = source.GetAttribute("Viewbox").Split(",");
            float viewboxLeft = float.Parse(viewboxContent[0]);
            float viewboxTop = float.Parse(viewboxContent[1]);
            float viewboxWidth = float.Parse(viewboxContent[2]);
            float viewboxHeight = float.Parse(viewboxContent[3]);

            this.Viewbox = new SKRect((int)viewboxLeft, (int)viewboxTop, (int)viewboxWidth, (int)viewboxHeight);

            string[] viewportContent = source.GetAttribute("Viewport").Split(",");
            float viewportLeft = float.Parse(viewportContent[0]);
            float viewportTop = float.Parse(viewportContent[1]);
            float viewportWidth = float.Parse(viewportContent[2]);
            float viewportHeight = float.Parse(viewportContent[3]);

            this.Viewport = new SKRect((int)viewportLeft, (int)viewportTop, (int)viewportWidth, (int)viewportHeight);
            source.ReadToDescendant("MatrixTransform");
            this.text_matrix = source.GetAttribute("Matrix");

            OnParsed(zip_file);
        }

        private void OnParsed(Java.Util.Zip.ZipFile zip_file)
        {

            this.InternalPaint.Style = SKPaintStyle.Fill;

            int Xdensity = 96;
            int Ydensity = 96;
            if (this.ImageSource.Split("/")[3].Split(".")[1] == "jpg" || this.ImageSource.Split("/")[3].Split(".")[1] == "jpeg")
            {
                AssetManager assets = Android.App.Application.Context.Assets;
                Stream image_header_stream = zip_file.GetInputStream(zip_file.GetEntry(this.ImageSource.Substring(1)));
                BinaryReader reader = new BinaryReader(image_header_stream);
                _ = reader.ReadBytes(13);
                if (reader.ReadByte() == 1)
                {
                    Xdensity = (reader.ReadByte() << 8 | reader.ReadByte()) & 0xFFFF;
                    Ydensity = (reader.ReadByte() << 8 | reader.ReadByte()) & 0xFFFF;
                }
                reader.Dispose();
            }


            float viewboxLeft = this.Viewbox.Left;
            float viewboxTop = this.Viewbox.Top;
            float viewboxWidth = this.Viewbox.Width;
            float viewboxHeight = this.Viewbox.Height;

            float viewportLeft = this.Viewport.Left;
            float viewportTop = this.Viewport.Top;
            float viewportWidth = this.Viewport.Width;
            float viewportHeight = this.Viewport.Height;

            float scaleX = Xdensity / 96F;
            float scaleY = Ydensity / 96F;

            SKRect ViewBox = new SKRect(viewboxLeft * scaleX, viewboxTop * scaleY, (viewboxLeft + viewboxWidth) * scaleX, (viewboxTop + viewboxHeight) * scaleY);
            SKRect ViewPort = new SKRect(viewportLeft, viewportTop, viewportLeft + viewportWidth, viewportTop + viewportHeight);
            Stream xps_image_stream = zip_file.GetInputStream(zip_file.GetEntry(this.ImageSource.Substring(1)));
            var bmp = SKBitmap.Decode(xps_image_stream);
            SKRectI bitmapBox = SKRectI.Round(ViewBox);
            var bmp0 = new SKBitmap(bitmapBox.Width, bitmapBox.Height);
            bmp.ExtractSubset(bmp0, bitmapBox);

            string[] matrix_number = text_matrix.Split(",");
            SKMatrix LocalMatrix = new SKMatrix(float.Parse(matrix_number[0]), float.Parse(matrix_number[2]), float.Parse(matrix_number[4]), float.Parse(matrix_number[1]), float.Parse(matrix_number[3]), float.Parse(matrix_number[5]), 0, 0, 1);
            SKMatrix PreScale = SKMatrix.CreateScale(ViewPort.Width / (float)ViewBox.Width, ViewPort.Height / (float)ViewBox.Height);
            LocalMatrix = LocalMatrix.PreConcat(PreScale);
            this.InternalPaint.Shader = SKShader.CreateBitmap(bmp0, SKShaderTileMode.Decal, SKShaderTileMode.Decal, LocalMatrix);
        }
    }


    /*
    public class XPSImageBrush : XPSTileBrush
    {
        public string ImageSource { get; set; }

        string text_matrix;

        public SKCanvas canvas;
        public XmlReader source;
        public Java.Util.Zip.ZipFile zip_file;
        public SKPaint ImagePaint;
        public SKShaderTileMode tile_mode_x;
        public SKShaderTileMode tile_mode_y;



        public XPSImageBrush(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {
            Parse(source, zip_file);
        }

        private void Parse(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {

            this.ImageSource = source.GetAttribute("ImageSource");

            string[] viewboxContent = source.GetAttribute("Viewbox").Split(",");
            float viewboxLeft = float.Parse(viewboxContent[0]);
            float viewboxTop = float.Parse(viewboxContent[1]);
            float viewboxWidth = float.Parse(viewboxContent[2]);
            float viewboxHeight = float.Parse(viewboxContent[3]);

            this.Viewbox = new SKRect((int)viewboxLeft, (int)viewboxTop, (int)(viewboxLeft+viewboxWidth), (int)(viewboxTop + viewboxHeight));

            string[] viewportContent = source.GetAttribute("Viewport").Split(",");
            float viewportLeft = float.Parse(viewportContent[0]);
            float viewportTop = float.Parse(viewportContent[1]);
            float viewportWidth = float.Parse(viewportContent[2]);
            float viewportHeight = float.Parse(viewportContent[3]);

            this.Viewport = new SKRect(viewportLeft, viewportTop,(viewportLeft+viewportWidth),(viewportTop+viewportHeight));

            string tile_mode_string = source.GetAttribute("TileMode");

            source.ReadToDescendant("MatrixTransform");
            this.text_matrix = source.GetAttribute("Matrix");

            


            if (tile_mode_string == "None")
            {
                this.tile_mode_x = SKShaderTileMode.Decal;
                this.tile_mode_y = SKShaderTileMode.Decal;
            }
            else if ( tile_mode_string == "FlipX")
            {
                this.tile_mode_x = SKShaderTileMode.Mirror;
                this.tile_mode_y = SKShaderTileMode.Decal;
            }
            else if (tile_mode_string == "FlipY")
            {
                this.tile_mode_x = SKShaderTileMode.Decal;
                this.tile_mode_y = SKShaderTileMode.Mirror;
            }
            else if (tile_mode_string == "FlipXY")
            {
                this.tile_mode_x = SKShaderTileMode.Mirror;
                this.tile_mode_y = SKShaderTileMode.Mirror;
            }
            else if (tile_mode_string == "Tile")
            {
                this.tile_mode_x = SKShaderTileMode.Repeat;
                this.tile_mode_y = SKShaderTileMode.Repeat;
            }

            OnParsed(zip_file);
        }

        private void OnParsed(Java.Util.Zip.ZipFile zip_file)
        {

            this.InternalPaint.Style = SKPaintStyle.Fill;

            int Xdensity = 96;
            int Ydensity = 96;
            if (this.ImageSource.Split("/")[3].Split(".")[1] == "jpg" || this.ImageSource.Split("/")[3].Split(".")[1] == "jpeg")
            {
                AssetManager assets = Android.App.Application.Context.Assets;
                Stream image_header_stream = zip_file.GetInputStream(zip_file.GetEntry(this.ImageSource.Substring(1)));
                BinaryReader reader = new BinaryReader(image_header_stream);
                _ = reader.ReadBytes(13);
                if (reader.ReadByte() == 1)
                {
                    Xdensity = (reader.ReadByte() << 8 | reader.ReadByte()) & 0xFFFF;
                    Ydensity = (reader.ReadByte() << 8 | reader.ReadByte()) & 0xFFFF;
                }
                reader.Dispose();
            }



            float viewboxLeft_xps =this.Viewbox.Left;
            float viewboxTop_xps = this.Viewbox.Top;
            float viewboxWidth_xps = this.Viewbox.Width;
            float viewboxHeight_xps = this.Viewbox.Height;

            float viewportLeft_xps = this.Viewport.Left;
            float viewportTop_xps = this.Viewport.Top;
            float viewportWidth_xps = this.Viewport.Width;
            float viewportHeight_xps = this.Viewport.Height;


            float scaleX = Xdensity / 96F;
            float scaleY = Ydensity / 96F;

            SKRect ViewBox = new SKRect(viewboxLeft_xps * scaleX, viewboxTop_xps * scaleY, (viewboxLeft_xps + viewboxWidth_xps) * scaleX, (viewboxTop_xps + viewboxHeight_xps) * scaleY);
            SKRect ViewPort = new SKRect(viewportLeft_xps, viewportTop_xps, viewportLeft_xps + viewportWidth_xps, viewportTop_xps + viewportHeight_xps);


            Stream xps_image_stream = zip_file.GetInputStream(zip_file.GetEntry(this.ImageSource.Substring(1)));
            var bmp = SKBitmap.Decode(xps_image_stream);

            //SKRect cut_view_box = new SKRect(Viewbox.Left, Viewbox.Top, Viewbox.Width / 3, Viewbox.Height / 3);
            SKRectI bitmapBox = SKRectI.Round(ViewBox);
            var bmp0 = new SKBitmap(bitmapBox.Width, bitmapBox.Height);

            bmp.ExtractSubset(bmp0, bitmapBox);

            string[] matrix_number = text_matrix.Split(",");
            SKMatrix LocalMatrix = new SKMatrix(float.Parse(matrix_number[0]), float.Parse(matrix_number[2]), float.Parse(matrix_number[4]), float.Parse(matrix_number[1]), float.Parse(matrix_number[3]), float.Parse(matrix_number[5]), 0, 0, 1);

            SKMatrix PreScale = SKMatrix.CreateScale(viewportWidth_xps / viewboxWidth_xps, viewportHeight_xps / viewboxHeight_xps); 
            LocalMatrix = LocalMatrix.PreConcat(PreScale);

            SKPictureRecorder record = new SKPictureRecorder();
            SKCanvas recording_anvas = record.BeginRecording(ViewBox);
            recording_anvas.DrawBitmap(bmp0, 0, 0);
            SKPicture picture = record.EndRecording();

            this.InternalPaint.Shader = SKShader.CreatePicture(picture, this.tile_mode_x, this.tile_mode_x, LocalMatrix, ViewBox);
        }

        public SKPaint return_paint()
        {
            return this.InternalPaint;
        }
    }
    //*/


    public class XPSVisualBrush : XPSTileBrush
    {
        public bool AutoLayoutContent { get; set; }
        public XPSVisual Visual { get; set; }

        public SKShaderTileMode tile_mode_x;
        public SKShaderTileMode tile_mode_y;
        public string Data;
        public XPSBrush linear_gradient_brush;
        public SKPath path;
        public string RenderTransform;
        public XPSPath xps_path;
        public XPSGlyphs xps_glyphs;
        public string glyphs_signal;

        public XPSVisualBrush(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {
            Parse(source, zip_file);
        }

        private void Parse(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {
            string[] viewboxContent = source.GetAttribute("Viewbox").Split(",");
            float viewboxLeft = float.Parse(viewboxContent[0]);
            float viewboxTop = float.Parse(viewboxContent[1]);
            float viewboxWidth = float.Parse(viewboxContent[2]);
            float viewboxHeight = float.Parse(viewboxContent[3]);

            //SKRect is the left top right bottom
            this.Viewbox = new SKRect((int)viewboxLeft, (int)viewboxTop, (int)(viewboxLeft + viewboxWidth), (int)(viewboxTop + viewboxHeight));

            string[] viewportContent = source.GetAttribute("Viewport").Split(",");
            float viewportLeft = float.Parse(viewportContent[0]);
            float viewportTop = float.Parse(viewportContent[1]);
            float viewportWidth = float.Parse(viewportContent[2]);
            float viewportHeight = float.Parse(viewportContent[3]);

            this.Viewport = new SKRect(viewportLeft, viewportTop, (viewportLeft + viewportWidth), (viewportTop + viewportHeight));

            string tile_mode_string = source.GetAttribute("TileMode");

            if (tile_mode_string == "None")
            {
                this.tile_mode_x = SKShaderTileMode.Decal;
                this.tile_mode_y = SKShaderTileMode.Decal;
            }
            else if (tile_mode_string == "FlipX")
            {
                this.tile_mode_x = SKShaderTileMode.Mirror;
                this.tile_mode_y = SKShaderTileMode.Decal;
            }
            else if (tile_mode_string == "FlipY")
            {
                this.tile_mode_x = SKShaderTileMode.Decal;
                this.tile_mode_y = SKShaderTileMode.Mirror;
            }
            else if (tile_mode_string == "FlipXY")
            {
                this.tile_mode_x = SKShaderTileMode.Mirror;
                this.tile_mode_y = SKShaderTileMode.Mirror;
            }
            else if (tile_mode_string == "Tile")
            {
                this.tile_mode_x = SKShaderTileMode.Repeat;
                this.tile_mode_y = SKShaderTileMode.Repeat;
            }

            source.ReadToDescendant("Path");
            this.Data = source.GetAttribute("Data");
            this.path = SKPath.ParseSvgPathData(this.Data);

            if (source.Name == "Path" && source.GetAttribute("Data") != null)
            {
                this.xps_path = new XPSPath(source, zip_file);
                //xps_path.Draw(canvas);
            }

            if (source.Name == "Canvas")
            {
                if (source.ReadToDescendant("Glyphs"))
                {
                    glyphs_signal = "Glyphs";
                    this.xps_glyphs = new XPSGlyphs(source, zip_file);
                }
            }

            //this.RenderTransform = source.GetAttribute("RenderTransform");

            //source.ReadToDescendant("LinearGradientBrush");
            //this.linear_gradient_brush = new XPSLinearGradientBrush(source,zip_file);

            OnParsed(zip_file);
        }

        private void OnParsed(Java.Util.Zip.ZipFile zip_file)
        {
            this.InternalPaint.Style = SKPaintStyle.Fill;

            float viewboxLeft_xps = this.Viewbox.Left;
            float viewboxTop_xps = this.Viewbox.Top;
            float viewboxWidth_xps = this.Viewbox.Width;
            float viewboxHeight_xps = this.Viewbox.Height;

            float viewportLeft_xps = this.Viewport.Left;
            float viewportTop_xps = this.Viewport.Top;
            float viewportWidth_xps = this.Viewport.Width;
            float viewportHeight_xps = this.Viewport.Height;



            SKRect ViewBox = new SKRect(viewboxLeft_xps , viewboxTop_xps , (viewboxLeft_xps + viewboxWidth_xps) , (viewboxTop_xps + viewboxHeight_xps) );
            SKMatrix pre_scale_matrix = SKMatrix.CreateScale(viewportWidth_xps / viewboxWidth_xps, viewportHeight_xps / viewboxHeight_xps);


            SKPictureRecorder record = new SKPictureRecorder();
            SKCanvas recording_anvas = record.BeginRecording(ViewBox);

            //SKMatrix transform_matrix_geometry = new SKMatrix(float.Parse(RenderTransform.Split(",")[0]), float.Parse(RenderTransform.Split(",")[2]), float.Parse(RenderTransform.Split(",")[4]), float.Parse(RenderTransform.Split(",")[1]), float.Parse(RenderTransform.Split(",")[3]), float.Parse(RenderTransform.Split(",")[5]), 0, 0, 1);
            //SKMatrix pre_matrix = recording_anvas.TotalMatrix;
            //SKMatrix post_matrix = SKMatrix.CreateIdentity();
            //SKMatrix.Concat(ref post_matrix, recording_anvas.TotalMatrix, transform_matrix_geometry);

            //recording_anvas.SetMatrix(post_matrix);
            //recording_anvas.DrawPath(this.path, this.linear_gradient_brush.ToPaint());
            if (glyphs_signal == "Glyphs")
            {
                xps_glyphs.Draw(recording_anvas);
            }
            else
            {
                xps_path.Draw(recording_anvas);
            }
            //recording_anvas.SetMatrix(pre_matrix);
            SKPicture picture = record.EndRecording();

            this.InternalPaint.Shader = SKShader.CreatePicture(picture, this.tile_mode_x, this.tile_mode_x, pre_scale_matrix, ViewBox);
        }
    }

    public class XPSGradientBrush : XPSBrush
    {
        public string ColorInterpolationMode { get; set; }
        public string GradientStops { get; set; }
        public string MappingMode { get; set; }
        public string SpreadMethod { get; set; }

    }

    public class XPSLinearGradientBrush : XPSGradientBrush
    {
        public SKPoint StartPoint { get; set; }
        public SKPoint EndPoint { get; set; }

        public SKShaderTileMode TileMode;

        public XPSLinearGradientBrush(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {
            Parse(source, zip_file);
        }

        private void Parse(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {
            this.MappingMode = source.GetAttribute("MappingMode");
            this.StartPoint = new SKPoint(float.Parse(source.GetAttribute("StartPoint").Split(",")[0]), float.Parse(source.GetAttribute("StartPoint").Split(",")[1]));
            this.EndPoint = new SKPoint(float.Parse(source.GetAttribute("EndPoint").Split(",")[0]), float.Parse(source.GetAttribute("EndPoint").Split(",")[1]));
            this.SpreadMethod = source.GetAttribute("SpreadMethod");
            source.ReadToDescendant("LinearGradientBrush.GradientStops");

            List<SKColor> color_list = new List<SKColor>();
            List<float> offset_list = new List<float>();
            

            if (this.SpreadMethod == "Pad")
            {
                this.TileMode = SKShaderTileMode.Clamp;
            }
            else if (this.SpreadMethod == "Repeat")
            {
                this.TileMode = SKShaderTileMode.Repeat;
            }
            else if (this.SpreadMethod == "Reflect")
            {
                this.TileMode = SKShaderTileMode.Mirror;
            }

            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "GradientStop")
                    {

                        if (source.GetAttribute("Color").Length == 7)
                        {
                            color_list.Add(new SKColor(uint.Parse("FF" + source.GetAttribute("Color").Substring(1), System.Globalization.NumberStyles.HexNumber)));
                        }
                        else if (source.GetAttribute("Color").Length == 9)
                        {
                            color_list.Add(new SKColor(uint.Parse(source.GetAttribute("Color").Substring(1), System.Globalization.NumberStyles.HexNumber)));
                        }

                        offset_list.Add(float.Parse(source.GetAttribute("Offset")));
                    }

                    if (source.Name == "LinearGradientBrush.GradientStops" && source.NodeType == XmlNodeType.EndElement)
                    {
                        break;
                    }
                }
            }
            this.InternalPaint.Shader = SKShader.CreateLinearGradient( this.StartPoint, this.EndPoint, color_list.ToArray(), offset_list.ToArray(), this.TileMode );
        }
    }

    public class XPSRadialGradientBrush : XPSGradientBrush
    {
        public SKPoint Center { get; set; }
        public SKPoint GradientOrigin { get; set; }
        public float RadiusX { get; set; }
        public float RadiusY { get; set; }

        public SKShaderTileMode TileMode;

        public XPSRadialGradientBrush(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {
            Parse(source, zip_file);
        }

        private void Parse(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {
            this.MappingMode = source.GetAttribute("MappingMode");
            this.Center =  new SKPoint(float.Parse(source.GetAttribute("Center").Split(",")[0]), float.Parse(source.GetAttribute("Center").Split(",")[1]));
            this.GradientOrigin = new SKPoint(float.Parse(source.GetAttribute("GradientOrigin").Split(",")[0]), float.Parse(source.GetAttribute("GradientOrigin").Split(",")[1])); 
            this.RadiusX = float.Parse(source.GetAttribute("RadiusX"));
            this.RadiusY = float.Parse(source.GetAttribute("RadiusY"));
            this.SpreadMethod = source.GetAttribute("SpreadMethod");

            
            if (this.SpreadMethod == "Pad")
            {
                this.TileMode = SKShaderTileMode.Clamp;
            }
            else if (this.SpreadMethod == "Repeat")
            {
                this.TileMode = SKShaderTileMode.Repeat;
            }
            else if (this.SpreadMethod == "Reflect")
            {
                this.TileMode = SKShaderTileMode.Mirror;
            }


            source.ReadToDescendant("RadialGradientBrush.GradientStops");
            List<SKColor> color_list = new List<SKColor>();
            List<float> offset_list = new List<float>();

            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "GradientStop")
                    {

                        if (source.GetAttribute("Color").Length == 7)
                        {
                            color_list.Add(new SKColor(uint.Parse("FF" + source.GetAttribute("Color").Substring(1), System.Globalization.NumberStyles.HexNumber)));
                        }
                        else if (source.GetAttribute("Color").Length == 9)
                        {
                            color_list.Add(new SKColor(uint.Parse(source.GetAttribute("Color").Substring(1), System.Globalization.NumberStyles.HexNumber)));
                        }
                        
                        offset_list.Add(float.Parse(source.GetAttribute("Offset")));
                    }

                    if (source.Name == "RadialGradientBrush.GradientStops" && source.NodeType == XmlNodeType.EndElement)
                    {
                        break;
                    }
                }
            }

            SKMatrix gradient_matrix = new SKMatrix();
            float ScaleY = this.RadiusY / this.RadiusX;
            gradient_matrix = new SKMatrix(1, 0, 0, 0, ScaleY, -(ScaleY - 1) * Center.Y, 0, 0, 1);

            this.InternalPaint.Shader = SKShader.CreateTwoPointConicalGradient( this.GradientOrigin, 0, this.Center, this.RadiusX, color_list.ToArray(), offset_list.ToArray(), this.TileMode, gradient_matrix);
        }
    }


    public class XPSGeometry : XPSAnimatable
    {
        public SKRect Bounds { get; }
        public static XPSGeometry Empty { get; }
        public static double StandardFlatteningTolerance { get; }
        public XPSTransform Transform { get; set; }
    }

    public class XPSPathSegment : XPSAnimatable
    {
        public bool IsSmoothJoin { get; set; }
        public bool IsStroked { get; set; }

    }

    public class XPSArcSegment : XPSPathSegment
    {
        public bool IsLargeArc { get; set; }
        public SKPoint Point { get; set; }
        public double RotationAngle { get; set; }
        public SKSize Size { get; set; }
        public string SweepDirection { get; set; }

    }

    public class XPSBezierSegment : XPSPathSegment
    {
        public SKPoint Point1 { get; set; }
        public SKPoint Point2 { get; set; }
        public SKPoint Point3 { get; set; }

    }

    public class XPSLineSegment : XPSPathSegment
    {
        public SKPoint Point { get; set; }

    }

    public class XPSPolyBezierSegment : XPSPathSegment
    {
        public List<SKPoint> Points;
    }

    public class XPSPolyLineSegment : XPSPathSegment
    {
        public List<SKPoint> Points;

    }

    public class XPSPolyQuadraticBezierSegment : XPSPathSegment
    {
        public List<SKPoint> Points;

    }

    public class XPSQuadraticBezierSegment : XPSPathSegment
    {
        public SKPoint Point1 { get; set; }
        public SKPoint Point2 { get; set; }

    }

    public class XPSPathFigure : XPSAnimatable
    {
        public bool IsClosed { get; set; }
        public bool IsFilled { get; set; }

        public List<XPSPathSegment> Segments = new List<XPSPathSegment>(0);

        public SKPoint StartPoint;
        public SKPoint LastPoint;

        public SKPath InternalData;

        public SKPath Data => InternalData;


        public void parse_PathFigure(XmlReader source)
        {
            if (source.NodeType == XmlNodeType.Element)
            {
                if (source.Name == "PathFigure" && source.NodeType == XmlNodeType.Element)
                {
                    //Console.WriteLine("<" + source.Name + ">");
                    this.StartPoint = new SKPoint(float.Parse(source.GetAttribute("StartPoint").Split(",")[0]), float.Parse(source.GetAttribute("StartPoint").Split(",")[1]));

                }
            }


            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "PolyLineSegment" && source.NodeType == XmlNodeType.Element)
                    {
                        string[] points_in_string = source.GetAttribute("Points").Split(" ");
                        XPSPolyLineSegment PolyLine_one = new XPSPolyLineSegment();
                        PolyLine_one.Points = new List<SKPoint>();

                        for (int i = 0; i < points_in_string.Length; i++)
                        {
                            PolyLine_one.Points.Add(new SKPoint(float.Parse(points_in_string[i].Split(",")[0]), float.Parse(points_in_string[i].Split(",")[1])));
                        }

                        this.Segments.Add(PolyLine_one);
                    }

                    if (source.Name == "ArcSegment" && source.NodeType == XmlNodeType.Element )
                    {
                        XPSArcSegment Arc = new XPSArcSegment();
                        Arc.Size = new SKSize(float.Parse(source.GetAttribute("Size").Split(",")[0]), float.Parse(source.GetAttribute("Size").Split(",")[1]));
                        Arc.RotationAngle = float.Parse(source.GetAttribute("RotationAngle"));
                        Arc.IsLargeArc = source.GetAttribute("IsLargeArc") == "true";
                        Arc.SweepDirection = source.GetAttribute("Counterclockwise");
                        Arc.Point = new SKPoint(float.Parse(source.GetAttribute("Point").Split(",")[0]), float.Parse(source.GetAttribute("Point").Split(",")[1]));

                        this.Segments.Add(Arc);
                    }

                    if (source.Name == "PolyBezierSegment" && source.NodeType == XmlNodeType.Element )
                    {
                        XPSPolyBezierSegment PolyBezier = new XPSPolyBezierSegment();
                        PolyBezier.Points = new List<SKPoint>();

                        string[] points_in_string = source.GetAttribute("Points").Split(" ");

                        for (int i = 0; i < points_in_string.Length; i++)
                        {
                            PolyBezier.Points.Add(new SKPoint(float.Parse(points_in_string[i].Split(",")[0]), float.Parse(points_in_string[i].Split(",")[1])));
                        }

                        this.Segments.Add(PolyBezier);
                    }

                    if (source.Name == "PolyQuadraticBezierSegment" && source.NodeType == XmlNodeType.Element )
                    {
                        XPSPolyQuadraticBezierSegment PolyQuadraticBezier = new XPSPolyQuadraticBezierSegment();
                        PolyQuadraticBezier.Points = new List<SKPoint>();

                        string[] points_in_string = source.GetAttribute("Points").Split(" ");

                        for (int i = 0; i < points_in_string.Length; i++)
                        {
                            PolyQuadraticBezier.Points.Add(new SKPoint(float.Parse(points_in_string[i].Split(",")[0]), float.Parse(points_in_string[i].Split(",")[1])));
                        }

                        this.Segments.Add(PolyQuadraticBezier);
                    }

                    if (source.Name == "PathFigure" && source.NodeType == XmlNodeType.EndElement )
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        source.Read();
                        break;
                    }
                }
            }
        }

        public void BuildInternalData()
        {
            
            LastPoint = new SKPoint(StartPoint.X, StartPoint.Y);
            InternalData = new SKPath();
            InternalData.MoveTo(LastPoint);

            foreach (XPSPathSegment Segment in Segments)
            {

                if (Segment is XPSArcSegment Arc)
                {
                    InternalData.ArcTo(
                            Arc.Size.Width,
                            Arc.Size.Height,
                            (float)Arc.RotationAngle,
                            Arc.IsLargeArc ? SKPathArcSize.Large : SKPathArcSize.Small,
                            Arc.SweepDirection == "Clockwise" ? SKPathDirection.Clockwise : SKPathDirection.CounterClockwise,
                            Arc.Point.X,
                            Arc.Point.Y
                        );
                }
                else if (Segment is XPSBezierSegment Bezier)
                {
                    InternalData.CubicTo(
                        Bezier.Point1.X,
                        Bezier.Point1.Y,
                        Bezier.Point2.X,
                        Bezier.Point2.Y,
                        Bezier.Point3.X,
                        Bezier.Point3.Y);

                    LastPoint = Bezier.Point3;
                }
                else if(Segment is XPSLineSegment Line)
                {
                    InternalData.LineTo(Line.Point);
                    LastPoint = Line.Point;
                }
                else if (Segment is XPSPolyBezierSegment PolyBezier)
                {
                    if (PolyBezier.Points.Count >= 3)
                    {
                        InternalData.CubicTo(
                            PolyBezier.Points[0].X,
                            PolyBezier.Points[0].Y,
                            PolyBezier.Points[1].X,
                            PolyBezier.Points[1].Y,
                            PolyBezier.Points[2].X,
                            PolyBezier.Points[2].Y);

                        LastPoint = PolyBezier.Points[2];
                    }

                    if (PolyBezier.Points.Count > 3
                        && PolyBezier.Points.Count % 3 == 0)
                    {
                        for (int i = 3; i < PolyBezier.Points.Count; i += 3)
                        {
                            InternalData.CubicTo(
                                PolyBezier.Points[i].X,
                                PolyBezier.Points[i].Y,
                                PolyBezier.Points[i + 1].X,
                                PolyBezier.Points[i + 1].Y,
                                PolyBezier.Points[i + 2].X,
                                PolyBezier.Points[i + 2].Y);

                            LastPoint = PolyBezier.Points[i + 2];
                        }
                    }
                }
                else if (Segment is XPSPolyLineSegment PolyLine )
                {
                    if (PolyLine.Points.Count <= 1)
                    {
                        InternalData.LineTo(
                            PolyLine.Points[0].X,
                            PolyLine.Points[0].Y);

                        LastPoint = PolyLine.Points[0];
                    }

                    if (PolyLine.Points.Count > 1)
                    {
                        for (int i = 0; i < PolyLine.Points.Count; i++)
                        {
                            InternalData.LineTo(
                                PolyLine.Points[i].X,
                                PolyLine.Points[i].Y);

                            LastPoint = PolyLine.Points[i];
                        }
                    }
                }
                else if (Segment is XPSPolyQuadraticBezierSegment PolyQuadraticBezier)
                {
                    if (PolyQuadraticBezier.Points.Count >= 2)
                    {
                        InternalData.QuadTo(
                            PolyQuadraticBezier.Points[0].X,
                            PolyQuadraticBezier.Points[0].Y,
                            PolyQuadraticBezier.Points[1].X,
                            PolyQuadraticBezier.Points[1].Y);

                        LastPoint = PolyQuadraticBezier.Points[1];
                    }

                    if (PolyQuadraticBezier.Points.Count > 2
                        && PolyQuadraticBezier.Points.Count % 2 == 0)
                    {
                        for (int i = 3; i < PolyQuadraticBezier.Points.Count; i += 3)
                        {
                            InternalData.QuadTo(
                                PolyQuadraticBezier.Points[i].X,
                                PolyQuadraticBezier.Points[i].Y,
                                PolyQuadraticBezier.Points[i + 1].X,
                                PolyQuadraticBezier.Points[i + 1].Y);

                            LastPoint = PolyQuadraticBezier.Points[i + 1];
                        }
                    }
                }
                else if (Segment is XPSQuadraticBezierSegment QuadraticBezier)
                {
                    InternalData.QuadTo(
                        QuadraticBezier.Point1.X,
                        QuadraticBezier.Point1.Y,
                        QuadraticBezier.Point2.X,
                        QuadraticBezier.Point2.Y);

                    LastPoint = QuadraticBezier.Point2;
                }
                else
                {
                    throw new NotSupportedException("Not supported segment type: " + Segment.GetType());
                }
            }
        }


        public void Draw(SKCanvas canvas, XPSBrush brush)
        {
            SKPaint paint = new SKPaint();
            paint.Style = SKPaintStyle.Stroke;
            paint.Color = new SKColor(0, 255, 0);
            paint.StrokeWidth = 1;


            canvas.DrawPath(InternalData, brush.ToPaint());
        }

    }
    public class XPSPathGeometry : XPSGeometry
    {

        public List<XPSPathFigure> Figures = new List<XPSPathFigure>(0);
        public string FillRule { get; set; }

        private SKPathFillType InternalFillRule;
        private readonly SKPath InternalData = new SKPath();

        public XPSPathGeometry()
        {
            if (FillRule == "EvenOdd")
            {
                InternalFillRule = SKPathFillType.EvenOdd;
            }
            else if (FillRule == "NonZero")
            {
                InternalFillRule = SKPathFillType.Winding;
            }
        }

        private void BuildInternalData()
        {
            InternalData.Reset();
            InternalData.FillType = InternalFillRule;
            foreach (XPSPathFigure Figure in Figures)
            {
                InternalData.AddPath(Figure.Data);
            }
        }

        public SKPath Data => InternalData;
    }






    [Activity(Label = "@string/app_name", Theme = "@style/AppTheme.NoActionBar", MainLauncher = true)]
    public class MainActivity : AppCompatActivity
    {
        private static Java.IO.File CreateCacheFile(Context C, string FileName)
        {
            Java.IO.InputStream Input = ((InputStreamInvoker)C.Assets.Open(FileName)).BaseInputStream;
            Java.IO.File CacheFile = new Java.IO.File(C.CacheDir, FileName);
            Java.IO.OutputStream Output = new Java.IO.FileOutputStream(CacheFile);
            int i;
            byte[] Stream = new byte[1024];
            while ((i = Input.Read(Stream)) != -1)
            {
                Output.Write(Stream, 0, i);
            }
            Input.Close();
            Output.Close();
            return CacheFile;
        }

        private List<float> GlyphLeftList = new List<float>();
        private List<float> list_indices = new List<float>();

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

        private static List<byte> get_file_from_xps(string file_path_in_xps, Dictionary<string, List<byte>> zip_files_in_xps)
        {
            return zip_files_in_xps[file_path_in_xps.Remove(0, 1)];
        }

        private static Dictionary<string, List<byte>> read_files_from_xps(string xps_file_name)
        {
            AssetManager assets = Android.App.Application.Context.Assets;
            List<byte> xps_file_list = new List<byte>();

            Dictionary<string, List<byte>> xps_file_dictionary = new Dictionary<string, List<byte>>();

            using (var assetStream = assets.Open(xps_file_name, Android.Content.Res.Access.Streaming))

            using (var zipStream = new Java.Util.Zip.ZipInputStream(assetStream))
            {
                Java.Util.Zip.ZipEntry zipEntry;
                while ((zipEntry = zipStream.NextEntry) != null)
                {
                    List<byte> template_list = extractFile(zipEntry, zipStream);
                    xps_file_dictionary.Add(zipEntry.Name, template_list);
                }

            }

            return xps_file_dictionary;
        }

        private static List<byte> extractFile( Java.Util.Zip.ZipEntry entry, Java.Util.Zip.ZipInputStream input_stream )
        {
            List<byte> byte_file = new List<byte>();
            try
            {
                byte[] buf = new byte[1024];
                int length;

                while ((length = input_stream.Read(buf, 0, buf.Length)) >= 0)
                {
                    for (int i = 0; i < length; i++)
                    {
                        byte_file.Add(buf[i]);
                    }
                }
            }
            catch (IOException ioex)
            {
                Console.WriteLine(ioex);
            }

            return byte_file;
        }


        public static Stream DeobfuscateXpsFont_from_stream(string input_font_path, Stream image_header_stream)
        {
            string XpsFontFilename = input_font_path.Split("/")[2];

            int length = 512 * 1024;
            byte[] dta = new byte[length];
            image_header_stream.Read(dta, 0, dta.Length);
            {
                string guid = new Guid(XpsFontFilename.Split('.')[0]).ToString("N");
                byte[] guidBytes = new byte[16];
                for (int i = 0; i < guidBytes.Length; i++)
                {
                    guidBytes[i] = Convert.ToByte(guid.Substring(i * 2, 2), 16);
                }

                for (int i = 0; i < 32; i++)
                {
                    int gi = guidBytes.Length - (i % guidBytes.Length) - 1;
                    dta[i] ^= guidBytes[gi];
                }
            }
            Stream stream = new MemoryStream(dta);
            return stream;
            
        }

        
        public static Stream DeobfuscateXpsFont(string input_font_path, List<byte> image_header_list)
        {
            string XpsFontFilename = input_font_path.Split("/")[2];
            byte[] dta;
            dta = image_header_list.ToArray();
            {
                string guid = new Guid(XpsFontFilename.Split('.')[0]).ToString("N");
                byte[] guidBytes = new byte[16];
                for (int i = 0; i < guidBytes.Length; i++)
                {
                    guidBytes[i] = Convert.ToByte(guid.Substring(i * 2, 2), 16);
                }

                for (int i = 0; i < 32; i++)
                {
                    int gi = guidBytes.Length - (i % guidBytes.Length) - 1;
                    dta[i] ^= guidBytes[gi];
                }
            }
            Stream stream = new MemoryStream(dta);
            return stream;

        }

        /*
        public void parse_FixedPage(XmlReader source)
        {
            while (source.Read())
            {
                if ( source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement )
                {
                    if (source.Name == "Canvas" && source.NodeType == XmlNodeType.Element)
                    {
                        parse_Canvas(source);
                    }

                    if (source.Name == "Canvas" && source.NodeType == XmlNodeType.EndElement)
                    {
                        break;
                    }
                }
            }
        }

        public void parse_Canvas(XmlReader source)
        {
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {

                    if (source.Name == "Canvas.RenderTransform" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine( "<" + source.Name + ">" );
                        parse_Canvas_RenderTransform(source);
                    }

                    if (source.Name == "Canvas.RenderTransform" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine( "</" + source.Name + ">" );
                    }

                    if (source.Name == "Canvas" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine("<" + source.Name + ">");
                        parse_sub_Canvas(source);
                    }

                    if (source.Name == "Canvas" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        break;
                    }


                    if (source.Name == "Path" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine("<" + source.Name + ">");
                        //parse_Path(source);
                        //Console.WriteLine("===========================source.MoveToContent()===================================");
                        //Console.WriteLine(source.MoveToContent());
                        XPSPath_for_parse xps_path_parse = new XPSPath_for_parse(source);
                        xps_path_parse.parse_Path(source);

                    }

                    if (source.Name == "Path" && source.NodeType == XmlNodeType.EndElement)
                    {
                        //Console.WriteLine("</" + source.Name + ">");
                    }
                }
            }
        }


        public void parse_sub_Canvas(XmlReader source)
        {

            //read the object GetAttribute
            if (source.Name == "Canvas" && source.NodeType == XmlNodeType.Element)
            {
                Console.WriteLine("<" + source.Name + ">");
                Console.WriteLine(source.GetAttribute("Clip"));

            }

            //read the object Elements
            while (source.Read())
            {
                if (source.Name == "Canvas" && source.NodeType == XmlNodeType.EndElement)
                {
                    Console.WriteLine("</" + source.Name + ">");
                    jump_end_element(source);
                    break;
                }
            }

        }
        public void parse_Path(XmlReader source)
        {
            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
            {
                if (source.Name == "Path" && source.NodeType == XmlNodeType.Element)
                {
                    Console.WriteLine("<" + source.Name + ">");
                    Console.WriteLine(source.GetAttribute("Data"));
                    Console.WriteLine(source.GetAttribute("Clip"));
                    Console.WriteLine(source.GetAttribute("AutomationProperties.HelpText"));
                    Console.WriteLine(source.GetAttribute("RenderTransform"));
                }
            }


            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "Path.Fill" && source.NodeType == XmlNodeType.Element)
                    {

                        //Console.WriteLine("===================here[start]=======================");
                        parse_Path_Fill(source);
                        //Console.WriteLine("===================here[end]=======================");
                    }

                    if (source.Name == "Path" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        jump_end_element(source);
                        break;
                    }
                }
            }
        }

        public void parse_Path_Fill(XmlReader source)
        {
            //read the object GetAttribute
            //Console.WriteLine("===================here[start][0]=======================");
            //Console.WriteLine(source.Name);
            if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
            {
                if (source.Name == "Path.Fill" && source.NodeType == XmlNodeType.Element)
                {
                    //Console.WriteLine("==========here==============");
                    Console.WriteLine("<" + source.Name + ">");
                    //parse_VisualBrush(source);
                }
            }


            //parse the object Elements
            while (source.Read())
            {
                if (source.Name == "VisualBrush" && source.NodeType == XmlNodeType.Element)
                {
                    parse_VisualBrush(source);
                }

                if (source.Name == "LinearGradientBrush" && source.NodeType == XmlNodeType.Element)
                {
                    parse_LinearGradientBrush(source);
                }

                if (source.Name == "RadialGradientBrush" && source.NodeType == XmlNodeType.Element)
                {
                    parse_RadialGradientBrush(source);
                }

                if (source.Name == "Path.Fill" && source.NodeType == XmlNodeType.EndElement)
                {
                    Console.WriteLine("</" + source.Name + ">");
                    jump_end_element(source);
                    break;
                }
                
            }
        }

        public void parse_VisualBrush(XmlReader source)
        {
            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
            {
                if (source.Name == "VisualBrush" && source.NodeType == XmlNodeType.Element)
                {
                    Console.WriteLine("<" + source.Name + ">");
                    Console.WriteLine(source.GetAttribute("ImageSource"));
                    Console.WriteLine(source.GetAttribute("Viewbox"));
                    Console.WriteLine(source.GetAttribute("TileMode"));
                    Console.WriteLine(source.GetAttribute("ViewboxUnits"));
                    Console.WriteLine(source.GetAttribute("ViewportUnits"));
                    Console.WriteLine(source.GetAttribute("Viewport"));
                }
            }


            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "Path" && source.NodeType == XmlNodeType.Element)
                    {
                        //Console.WriteLine("<" + source.Name + ">");
                        parse_Path(source);
                    }

                    if (source.Name == "VisualBrush" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        jump_end_element(source);
                        break;
                    }
                }
            }
        }


        public void parse_LinearGradientBrush(XmlReader source)
        {
            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
            {
                if (source.Name == "LinearGradientBrush" && source.NodeType == XmlNodeType.Element)
                {
                    Console.WriteLine("<" + source.Name + ">");
                    Console.WriteLine(source.GetAttribute("MappingMode"));
                    Console.WriteLine(source.GetAttribute("StartPoint"));
                    Console.WriteLine(source.GetAttribute("EndPoint"));
                    Console.WriteLine(source.GetAttribute("SpreadMethod"));
                }
            }

            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "LinearGradientBrush.GradientStops" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine("< GradientStop Color = #FF0000 Offset = 0.0 /> ");
                    }

                    if (source.Name == "LinearGradientBrush" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        jump_end_element(source);
                        break;
                    }
                }
            }
        }


        public void parse_RadialGradientBrush(XmlReader source)
        {
            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
            {
                if (source.Name == "RadialGradientBrush" && source.NodeType == XmlNodeType.Element)
                {
                    Console.WriteLine("<" + source.Name + ">");
                    Console.WriteLine(source.GetAttribute("MappingMode"));
                    Console.WriteLine(source.GetAttribute("Center"));
                    Console.WriteLine(source.GetAttribute("GradientOrigin"));
                    Console.WriteLine(source.GetAttribute("RadiusX"));
                    Console.WriteLine(source.GetAttribute("RadiusY"));
                    Console.WriteLine(source.GetAttribute("SpreadMethod"));

                }
            }


            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "RadialGradientBrush.GradientStops" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine("< GradientStop Color = #FF0000 Offset = 0.0 /> ");
                    }

                    if (source.Name == "RadialGradientBrush" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        jump_end_element(source);
                        break;
                    }
                }
            }
        }

        public void parse_Canvas_RenderTransform(XmlReader source)
        {
            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "MatrixTransform" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine(source.GetAttribute("Matrix"));
                        break;
                    }
                }
            }
        }

        public void jump_end_element(XmlReader source)
        {
            source.Read();
        }

        //*/


        private void OnPaintSurface(object sender, SKPaintSurfaceEventArgs e)
        {
            float CanvasScale = ((sender as SKCanvasView).Width - 12) / 794F;

            SKCanvas canvas = e.Surface.Canvas;
            canvas.Save();
            canvas.Translate(6, 6);
            canvas.Scale(CanvasScale, CanvasScale);
            SKPaint paint = new SKPaint();

            //Stream XmlStream = Assets.Open("1.fpage");
            Dictionary<string, List<byte>> zip_files_in_xps = read_files_from_xps("Doc2_2.xps");

            Stream XmlStream = new MemoryStream(get_file_from_xps("/Documents/1/Pages/1.fpage", zip_files_in_xps).ToArray());


            XmlDocument localInfo_Xml = new XmlDocument();
            localInfo_Xml.Load(XmlStream);
            XmlStream.Close();
            Java.Util.Zip.ZipFile zip_file = new Java.Util.Zip.ZipFile(CreateCacheFile(this, "Doc2_2.xps"));

            Stream xml_file_stream = zip_file.GetInputStream(zip_file.GetEntry("FixedDocSeq.fdseq"));

            XmlDocument FixedDocSeq = new XmlDocument();
            FixedDocSeq.Load(xml_file_stream);
            xml_file_stream.Close();
            XmlAttribute next_file_path = FixedDocSeq.DocumentElement.GetElementsByTagName("DocumentReference")[0].Attributes["Source"];

            Stream xml_file_stream_second = zip_file.GetInputStream(zip_file.GetEntry(next_file_path.Value.Substring(1)));
            XmlDocument FixedDoc = new XmlDocument();
            FixedDoc.Load(xml_file_stream_second);
            xml_file_stream_second.Close();
            XmlAttribute pcc = FixedDoc.DocumentElement.GetElementsByTagName("PageContent")[0].Attributes["Source"];
            string prefix = Path.GetDirectoryName(next_file_path.Value);

            Stream xml_file_stream_third = zip_file.GetInputStream(zip_file.GetEntry($"{prefix.Substring(1)}/{pcc.Value}"));



            XmlReader xml_reader = XmlReader.Create(xml_file_stream_third);

            if (xml_reader.Read())
            {
                XmlReader r = xml_reader.ReadSubtree();
                
                while (r.Read())
                {
                    if (r.NodeType == XmlNodeType.Element || r.NodeType == XmlNodeType.Text || r.NodeType == XmlNodeType.EndElement)
                    {
                        //Console.WriteLine("[" + r.Name + "]");
                        //Console.WriteLine(r.NodeType);

                        //if (r.NodeType == XmlNodeType.Element)
                        //{
                        //    Console.WriteLine("<" + r.Name + ">");
                        //}


                        //if (r.NodeType == XmlNodeType.EndElement)
                        //{
                        //    Console.WriteLine("</" + r.Name + ">");
                        //}

                        if ( r.Name == "FixedPage" && r.NodeType == XmlNodeType.Element)
                        {
                            //parse_FixedPage(r);

                            XPSFixedPage_for_parse xps_fixedpage_parse = new XPSFixedPage_for_parse(r);
                            xps_fixedpage_parse.parse_FixedPage(r, zip_file, canvas);
                        }

                        if (r.Name == "FixedPage" && r.NodeType == XmlNodeType.EndElement)
                        {
                            break;
                        }

                        /*
                        if (r.Name != string.Empty)
                        {
                            Console.Write("[r.Name]=>");
                            Console.Write(r.Name);
                        }


                        if (r.Name == "Canvas")
                        {
                            if (r.ReadToDescendant("Glyphs"))
                            {
                                XPSGlyphs xps_glyphs = new XPSGlyphs(r, zip_file);
                                xps_glyphs.Draw(canvas);
                            }
                        }

                        if (r.Name == "Path" && r.GetAttribute("Stroke") != null )
                        {
                            XPSPath xps_path = new XPSPath(r, zip_file);
                            xps_path.Draw(canvas);
                        }

                        if (r.Name == "Path" && r.GetAttribute("Data") != null)
                        {
                            XPSPath xps_path = new XPSPath(r, zip_file);
                            xps_path.Draw(canvas);
                        }

                        if ( r.Name == "Path" && r.GetAttribute("Clip") != null )
                        {
                            XPSPath xps_path = new XPSPath(r, zip_file);
                            xps_path.Draw(canvas);
                        }
                        //*/
                    }
                }
            }



            paint.Style = SKPaintStyle.Stroke;
            paint.Color = new SKColor(0, 0, 0);
            paint.StrokeWidth = 0;
            //paint.Shader = image_shader;
            canvas.DrawRect(0, 0, 794, 1123, paint);

            canvas.Flush();
            canvas.Restore();

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

    
    public abstract class XPSPartPase
    {

    }

    public abstract class XPSParsable : XPSPartPase
    {
        public XPSParsable(XmlReader Source)
        {

        }
        public void jump_end_element(XmlReader source)
        {
            source.Read();
        }

    }


    public abstract class XPSVisual_for_parse : XPSParsable
    {
        public XPSVisual_for_parse(XmlReader Source) : base(Source)
        {

        }
        public void Draw(SKCanvas canvas)
        {

        }

    }



    public class XPSFixedPage_for_parse : XPSVisual_for_parse
    {
        public XPSFixedPage_Data xps_fixed_page_data = new XPSFixedPage_Data();
        public XPSFixedPage_for_parse(XmlReader Source) : base(Source)
        {

        }

        public void parse_FixedPage(XmlReader source, Java.Util.Zip.ZipFile zip_file, SKCanvas canvas)
        {
            

            if (source.Name == "FixedPage" && source.NodeType == XmlNodeType.Element)
            {
                Console.WriteLine("<" + source.Name + ">");
                Console.WriteLine(source.GetAttribute("Clip"));
                this.xps_fixed_page_data.xmlns = source.GetAttribute("xmlns");
                this.xps_fixed_page_data.Width = source.GetAttribute("Width");
                this.xps_fixed_page_data.Height = source.GetAttribute("Height");
                this.xps_fixed_page_data.xml_lang = source.GetAttribute("xml:lang");

            }

            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "Canvas" && source.NodeType == XmlNodeType.Element)
                    {
                        XPSCanvas_for_parse xps_canvas_parse = new XPSCanvas_for_parse(source);
                        this.xps_fixed_page_data.xps_data_children.Add( xps_canvas_parse.parse_Canvas(source, zip_file, canvas) );
                    }

                    if (source.Name == "Canvas" && source.NodeType == XmlNodeType.EndElement)
                    {
                        break;
                    }
                }
            }
        }

        public void Draw(SKCanvas canvas)
        {

        }
    }


    public class XPSCanvas_for_parse : XPSPanel
    {
        public XPSCanvas_Data xps_canvas_data = new XPSCanvas_Data();

        public XPSCanvas_for_parse(XmlReader Source) : base(Source)
        {
        }

        public XPSCanvas_Data parse_Canvas(XmlReader source, Java.Util.Zip.ZipFile zip_file, SKCanvas canvas)
        {
            

            if (source.Name == "Canvas" && source.NodeType == XmlNodeType.Element)
            {
                Console.WriteLine("<" + source.Name + ">");
                Console.WriteLine(source.GetAttribute("Clip"));
                this.xps_canvas_data.Clip = source.GetAttribute("Clip");

            }


            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {

                    if (source.Name == "Canvas.RenderTransform" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine("<" + source.Name + ">");
                        this.xps_canvas_data.RenderTransform = parse_Canvas_RenderTransform(source);
                    }

                    if (source.Name == "Canvas.RenderTransform" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                    }

                    if (source.Name == "Glyphs" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine("<" + source.Name + ">");
                        Console.WriteLine(source.GetAttribute("Clip"));
                        //XPSGlyphs_for_parse xps_glyphs_parser = new XPSGlyphs_for_parse(source);
                        //this.xps_canvas_data.xps_data_children.Add( xps_glyphs_parser.parse_Glyphs(source) );
                        XPSGlyphs xps_glyphs = new XPSGlyphs(source,zip_file);
                        xps_glyphs.Draw(canvas);
                    }


                    if (source.Name == "Path" && source.NodeType == XmlNodeType.Element)
                    {
                        XPSPath_for_parse xps_path_parser = new XPSPath_for_parse(source);
                        XPSPath_Data xps_path_data = xps_path_parser.parse_Path(source, zip_file,canvas);
                        this.xps_canvas_data.xps_data_children.Add(xps_path_data);

                        if ( xps_path_data.Fill!=null || xps_path_data.Stroke !=null)
                        {
                            xps_path_parser.Draw(canvas);
                        }
                    }

                    if (source.Name == "Path" && source.NodeType == XmlNodeType.EndElement)
                    {
                        //Console.WriteLine("</" + source.Name + ">");
                    }

                    if (source.Name == "Canvas" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine("<" + source.Name + ">");
                        this.xps_canvas_data.xps_data_children.Add( parse_Canvas(source, zip_file, canvas) );
                    }

                    if (source.Name == "Canvas" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        jump_end_element(source);
                        break;
                    }
                }
            }

            return this.xps_canvas_data;
        }

        public string parse_Canvas_RenderTransform(XmlReader source)
        {
            string render_transform = "";
            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "MatrixTransform" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine(source.GetAttribute("Matrix"));
                        render_transform = source.GetAttribute("Matrix");
                        break;
                    }
                }
            }
            return render_transform;
        }

        public void Draw(SKCanvas canvas)
        {

        }
    }


    public class XPSGlyphs_for_parse : XPSVisual_for_parse
    {
        public XPSGlyphs_Data xps_glyphs_data = new XPSGlyphs_Data();
        public XPSGlyphs_for_parse(XmlReader Source) : base(Source)
        {

        }

        public XPSGlyphs_Data parse_Glyphs(XmlReader source)
        {
            
            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element)
            {
                if (source.Name == "Glyphs" && source.NodeType == XmlNodeType.Element)
                {
                    Console.WriteLine("<" + source.Name + ">");
                    Console.WriteLine(source.GetAttribute("Name"));
                    Console.WriteLine(source.GetAttribute("BidiLevel"));
                    Console.WriteLine(source.GetAttribute("Fill"));
                    Console.WriteLine(source.GetAttribute("FontUri"));
                    Console.WriteLine(source.GetAttribute("FontRenderingEmSize"));
                    Console.WriteLine(source.GetAttribute("StyleSimulations"));
                    Console.WriteLine(source.GetAttribute("OriginX"));
                    Console.WriteLine(source.GetAttribute("OriginY"));
                    Console.WriteLine(source.GetAttribute("UnicodeString"));
                    Console.WriteLine(source.GetAttribute("Indices"));
                    Console.WriteLine(source.GetAttribute("xml:lang"));

                    this.xps_glyphs_data.Name = source.GetAttribute("Name");
                    this.xps_glyphs_data.BidiLevel = source.GetAttribute("BidiLevel");
                    this.xps_glyphs_data.Fill = source.GetAttribute("Fill");
                    this.xps_glyphs_data.FontUri = source.GetAttribute("FontUri");
                    this.xps_glyphs_data.FontRenderingEmSize = source.GetAttribute("FontRenderingEmSize");
                    this.xps_glyphs_data.StyleSimulations = source.GetAttribute("StyleSimulations");
                    this.xps_glyphs_data.OriginX = source.GetAttribute("OriginX");
                    this.xps_glyphs_data.OriginY = source.GetAttribute("OriginY");
                    this.xps_glyphs_data.UnicodeString = source.GetAttribute("UnicodeString");
                    this.xps_glyphs_data.Indices = source.GetAttribute("Indices");
                    this.xps_glyphs_data.xml_lang = source.GetAttribute("xml:lang");

                }
            }


            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "Glyphs" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        jump_end_element(source);
                        break;
                    }
                }
            }

            return this.xps_glyphs_data;
        }

    }

    public class XPSPath_for_parse : XPSParsable
    {
        public XPSPath_Data xps_path_data = new XPSPath_Data();
        public XPSPathGeometry_for_parse xps_path_geometry_for_parse;
        public XPSPath_for_parse(XmlReader Source) : base(Source)
        {

        }

        public XPSPath_Data parse_Path(XmlReader source ,Java.Util.Zip.ZipFile zip_file, SKCanvas canvas)
        {

            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element)
            {
                if (source.Name == "Path" && source.NodeType == XmlNodeType.Element)
                {
                    Console.WriteLine("<" + source.Name + ">");
                    Console.WriteLine(source.GetAttribute("Data"));
                    Console.WriteLine(source.GetAttribute("Clip"));
                    Console.WriteLine(source.GetAttribute("AutomationProperties.HelpText"));
                    Console.WriteLine(source.GetAttribute("RenderTransform"));
                    Console.WriteLine(source.GetAttribute("Stroke"));
                    Console.WriteLine(source.GetAttribute("StrokeThickness"));

                    this.xps_path_data.Data = source.GetAttribute("Data");
                    this.xps_path_data.Clip = source.GetAttribute("Clip");
                    this.xps_path_data.RenderTransform = source.GetAttribute("RenderTransform");
                    this.xps_path_data.Stroke = source.GetAttribute("Stroke");
                    this.xps_path_data.StrokeThickness = source.GetAttribute("StrokeThickness");

                }
            }

            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "Path.Fill" && source.NodeType == XmlNodeType.Element)
                    {
                        XPSBrush_for_parse xps_brush_for_parse = new XPSBrush_for_parse(source);

                        xps_brush_for_parse.parse_Path_Fill( source, zip_file, canvas );

                        this.xps_path_data.Fill = xps_brush_for_parse;
                    }

                    if (source.Name == "Path.Data" && source.NodeType == XmlNodeType.Element)
                    {
                        this.xps_path_geometry_for_parse = new XPSPathGeometry_for_parse(source);

                        this.xps_path_geometry_for_parse.parse_PathGeometry(source);
                        this.xps_path_geometry_for_parse.signal = "Stroke";

                    }

                    if (source.Name == "Path" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        jump_end_element(source);
                        break;
                    }
                }
            }

            return this.xps_path_data;
        }


        public void Draw(SKCanvas canvas) 
        {
            if (xps_path_data.Fill != null)
            {
                if ((xps_path_data.Fill.xps_brush_data.signal == "LinearGradientBrush" || xps_path_data.Fill.xps_brush_data.signal == "RadialGradientBrush") && this.xps_path_data.RenderTransform != null)
                {
                    String RenderTransform = xps_path_data.RenderTransform;
                    SKMatrix transform_matrix_geometry = new SKMatrix(float.Parse(RenderTransform.Split(",")[0]), float.Parse(RenderTransform.Split(",")[2]), float.Parse(RenderTransform.Split(",")[4]), float.Parse(RenderTransform.Split(",")[1]), float.Parse(RenderTransform.Split(",")[3]), float.Parse(RenderTransform.Split(",")[5]), 0, 0, 1);
                    SKMatrix pre_matrix = canvas.TotalMatrix;
                    SKMatrix post_matrix = SKMatrix.CreateIdentity();
                    SKMatrix.Concat(ref post_matrix, canvas.TotalMatrix, transform_matrix_geometry);
                    canvas.SetMatrix(post_matrix);
                    canvas.DrawPath(SKPath.ParseSvgPathData(this.xps_path_data.Data), this.xps_path_data.Fill.xps_brush_data.InternalPaint);
                    canvas.SetMatrix(pre_matrix);

                }


                if (xps_path_data.Fill.xps_brush_data.signal == "VisualBrush")
                {
                    canvas.DrawPath(SKPath.ParseSvgPathData(this.xps_path_data.Data), this.xps_path_data.Fill.xps_brush_data.InternalPaint);
                }
            }


            if (this.xps_path_data.Stroke != null && this.xps_path_geometry_for_parse.signal == "Stroke")
            {
                for (int i = 0; i < this.xps_path_geometry_for_parse.Figures.Count; i++)
                {
                    
                    string Transformr = this.xps_path_geometry_for_parse.xps_path_geometry_data.Transform;
                    SKMatrix transform_matrix_geometry = new SKMatrix(float.Parse(Transformr.Split(",")[0]), float.Parse(Transformr.Split(",")[2]), float.Parse(Transformr.Split(",")[4]), float.Parse(Transformr.Split(",")[1]), float.Parse(Transformr.Split(",")[3]), float.Parse(Transformr.Split(",")[5]), 0, 0, 1);

                    SKMatrix pre_matrix = canvas.TotalMatrix;
                    SKMatrix post_matrix = SKMatrix.CreateIdentity();
                    SKMatrix.Concat(ref post_matrix, canvas.TotalMatrix, transform_matrix_geometry);
                    canvas.SetMatrix(post_matrix);

                    XPSSolidColorBrush brush = new XPSSolidColorBrush(new SKColor(uint.Parse(this.xps_path_data.Stroke.Substring(1), System.Globalization.NumberStyles.HexNumber)));
                    brush.InternalPaint.Style = SKPaintStyle.Stroke;
                    brush.InternalPaint.StrokeWidth = float.Parse(this.xps_path_data.StrokeThickness);
                    canvas.DrawPath(this.xps_path_geometry_for_parse.Figures[i].Data, brush.InternalPaint);
                    canvas.SetMatrix(pre_matrix);
                }
            }

            if (this.xps_path_data.Clip != null && this.xps_path_data.Data == null) //PathGeometry with RadialGradientBrush
            {
                for (int i = 0; i < this.xps_path_geometry_for_parse.Figures.Count; i++)
                {
                    canvas.DrawPath(this.xps_path_geometry_for_parse.Figures[i].Data, this.xps_path_data.Fill.xps_brush_data.InternalPaint);
                }
            }

            if (this.xps_path_data.Clip != null && this.xps_path_data.Data != null && this.xps_path_data.Fill.xps_brush_data.signal == "ImageBrush" )
            {
                canvas.DrawPath(SKPath.ParseSvgPathData(this.xps_path_data.Data), this.xps_path_data.Fill.xps_brush_data.InternalPaint);
            }
        }
    }

    public class XPSBrush_for_parse : XPSParsable
    {
        public XPSBrush_Data xps_brush_data = new XPSBrush_Data();
        public XPSBrush_for_parse(XmlReader Source) : base(Source)
        {

        }

        public XPSBrush_for_parse parse_Path_Fill(XmlReader source, Java.Util.Zip.ZipFile zip_file, SKCanvas canvas)
        {
            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element)
            {
                if (source.Name == "Path.Fill" && source.NodeType == XmlNodeType.Element)
                {

                    Console.WriteLine("<" + source.Name + ">");

                }
            }


            //parse the object Elements
            while (source.Read())
            {
                if (source.Name == "ImageBrush" && source.NodeType == XmlNodeType.Element)
                {
                    //XPSImageBrush_for_parse xps_image_brush_for_parse = new XPSImageBrush_for_parse(source);
                    //xps_image_brush_for_parse.parse_ImageBrush(source, zip_file);
                    //this.xps_brush_data.InternalPaint = xps_image_brush_for_parse.InternalPaint;
                    //this.xps_brush_data.signal = "ImageBrush";
                    //Console.WriteLine("=======================XPSImageBrush_for_parse===========================");
                    //return xps_image_brush_for_parse;


                    XPSImageBrush xps_image_brush = new XPSImageBrush(source, zip_file);
                    this.xps_brush_data.InternalPaint = xps_image_brush.InternalPaint;
                    this.xps_brush_data.signal = "ImageBrush";
                    return null;

                }

                if (source.Name == "VisualBrush" && source.NodeType == XmlNodeType.Element)
                {
                    //XPSVisualBrush_for_parse xps_visual_brush_parse = new XPSVisualBrush_for_parse(source);
                    //return xps_visual_brush_parse.parse_VisualBrush(source,canvas);

                    XPSVisualBrush_for_parse xps_visual_brush_parse = new XPSVisualBrush_for_parse(source);
                    xps_visual_brush_parse.parse_VisualBrush(source,zip_file,canvas);
                    this.xps_brush_data.InternalPaint = xps_visual_brush_parse.InternalPaint;
                    this.xps_brush_data.signal = "VisualBrush";

                    return xps_visual_brush_parse;
                }

                if (source.Name == "LinearGradientBrush" && source.NodeType == XmlNodeType.Element)
                {

                    XPSLinearGradientBrush_for_parse xps_linear_gradient_brush_parse = new XPSLinearGradientBrush_for_parse(source);
                    xps_linear_gradient_brush_parse.parse_LinearGradientBrush(source);
                    this.xps_brush_data.InternalPaint = xps_linear_gradient_brush_parse.InternalPaint;
                    this.xps_brush_data.signal = "LinearGradientBrush";

                    return xps_linear_gradient_brush_parse;
                }

                if (source.Name == "RadialGradientBrush" && source.NodeType == XmlNodeType.Element)
                {
                    XPSRadialGradientBrush_for_parse xps_radial_gradient_brush_parse = new XPSRadialGradientBrush_for_parse(source);
                    xps_radial_gradient_brush_parse.parse_RadialGradientBrush(source);
                    this.xps_brush_data.InternalPaint = xps_radial_gradient_brush_parse.InternalPaint;
                    this.xps_brush_data.signal = "RadialGradientBrush";

                    return xps_radial_gradient_brush_parse;
                }

                if (source.Name == "Path.Fill" && source.NodeType == XmlNodeType.EndElement)
                {
                    Console.WriteLine("</" + source.Name + ">");
                    jump_end_element(source);
                    break;
                }

            }

            return null;
        }
    }



    public class XPSImageBrush_for_parse : XPSBrush_for_parse
    {
        XPSImageBrush_Data xps_image_brush_data = new XPSImageBrush_Data();
        public SKPaint InternalPaint = new SKPaint();
        public string ImageSource { get; set; }

        string text_matrix;

        public SKCanvas canvas;
        public XmlReader source;
        public Java.Util.Zip.ZipFile zip_file;
        public SKPaint ImagePaint;
        public SKShaderTileMode tile_mode_x;
        public SKShaderTileMode tile_mode_y;

        public SKRect Viewbox;
        public SKRect Viewport;

        public XPSImageBrush_for_parse(XmlReader source) : base(source)
        {
            //Parse(source, zip_file);
        }

        public void parse_ImageBrush(XmlReader source, Java.Util.Zip.ZipFile zip_file)
        {
            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element)
            {
                if (source.Name == "ImageBrush" && source.NodeType == XmlNodeType.Element)
                {
                    Console.WriteLine("<" + source.Name + ">");
                    Console.WriteLine(source.GetAttribute("ImageSource"));
                    Console.WriteLine(source.GetAttribute("Viewbox"));
                    Console.WriteLine(source.GetAttribute("TileMode"));
                    Console.WriteLine(source.GetAttribute("ViewboxUnits"));
                    Console.WriteLine(source.GetAttribute("ViewportUnits"));
                    Console.WriteLine(source.GetAttribute("Viewport"));

                    this.xps_image_brush_data.ImageSource = source.GetAttribute("ImageSource");
                    this.xps_image_brush_data.Viewbox = source.GetAttribute("Viewbox");
                    this.xps_image_brush_data.TileMode = source.GetAttribute("TileMode");
                    this.xps_image_brush_data.ViewboxUnits = source.GetAttribute("ViewboxUnits");
                    this.xps_image_brush_data.ViewportUnits = source.GetAttribute("ViewportUnits");
                    this.xps_image_brush_data.Viewport = source.GetAttribute("Viewport");

                }
            }

            Console.WriteLine();
            this.ImageSource = source.GetAttribute("ImageSource");

            string[] viewboxContent = source.GetAttribute("Viewbox").Split(",");
            float viewboxLeft = float.Parse(viewboxContent[0]);
            float viewboxTop = float.Parse(viewboxContent[1]);
            float viewboxWidth = float.Parse(viewboxContent[2]);
            float viewboxHeight = float.Parse(viewboxContent[3]);

            this.Viewbox = new SKRect((int)viewboxLeft, (int)viewboxTop, (int)(viewboxLeft + viewboxWidth), (int)(viewboxTop + viewboxHeight));

            string[] viewportContent = source.GetAttribute("Viewport").Split(",");
            float viewportLeft = float.Parse(viewportContent[0]);
            float viewportTop = float.Parse(viewportContent[1]);
            float viewportWidth = float.Parse(viewportContent[2]);
            float viewportHeight = float.Parse(viewportContent[3]);

            this.Viewport = new SKRect(viewportLeft, viewportTop, (viewportLeft + viewportWidth), (viewportTop + viewportHeight));

            string tile_mode_string = source.GetAttribute("TileMode");

            source.ReadToDescendant("MatrixTransform");
            this.text_matrix = source.GetAttribute("Matrix");
            Console.WriteLine("========================parse_ImageBrush=========================");
            Console.WriteLine(this.ImageSource);
            Console.WriteLine(this.Viewbox);
            Console.WriteLine(this.Viewport);
            Console.WriteLine(this.text_matrix);



            if (tile_mode_string == "None")
            {
                this.tile_mode_x = SKShaderTileMode.Decal;
                this.tile_mode_y = SKShaderTileMode.Decal;
            }
            else if (tile_mode_string == "FlipX")
            {
                this.tile_mode_x = SKShaderTileMode.Mirror;
                this.tile_mode_y = SKShaderTileMode.Decal;
            }
            else if (tile_mode_string == "FlipY")
            {
                this.tile_mode_x = SKShaderTileMode.Decal;
                this.tile_mode_y = SKShaderTileMode.Mirror;
            }
            else if (tile_mode_string == "FlipXY")
            {
                this.tile_mode_x = SKShaderTileMode.Mirror;
                this.tile_mode_y = SKShaderTileMode.Mirror;
            }
            else if (tile_mode_string == "Tile")
            {
                this.tile_mode_x = SKShaderTileMode.Repeat;
                this.tile_mode_y = SKShaderTileMode.Repeat;
            }


            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {

                    if (source.Name == "ImageBrush" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        jump_end_element(source);
                        break;
                    }
                }
            }

            OnParsed(zip_file);
        }

        private void OnParsed(Java.Util.Zip.ZipFile zip_file)
        {

            this.InternalPaint.Style = SKPaintStyle.Fill;

            int Xdensity = 96;
            int Ydensity = 96;
            if (this.ImageSource.Split("/")[3].Split(".")[1] == "jpg" || this.ImageSource.Split("/")[3].Split(".")[1] == "jpeg")
            {
                AssetManager assets = Android.App.Application.Context.Assets;
                Stream image_header_stream = zip_file.GetInputStream(zip_file.GetEntry(this.ImageSource.Substring(1)));
                BinaryReader reader = new BinaryReader(image_header_stream);
                _ = reader.ReadBytes(13);
                if (reader.ReadByte() == 1)
                {
                    Xdensity = (reader.ReadByte() << 8 | reader.ReadByte()) & 0xFFFF;
                    Ydensity = (reader.ReadByte() << 8 | reader.ReadByte()) & 0xFFFF;
                }
                reader.Dispose();
            }



            float viewboxLeft_xps = this.Viewbox.Left;
            float viewboxTop_xps = this.Viewbox.Top;
            float viewboxWidth_xps = this.Viewbox.Width;
            float viewboxHeight_xps = this.Viewbox.Height;

            float viewportLeft_xps = this.Viewport.Left;
            float viewportTop_xps = this.Viewport.Top;
            float viewportWidth_xps = this.Viewport.Width;
            float viewportHeight_xps = this.Viewport.Height;


            float scaleX = (float)2.291667 * 3;//Xdensity / 96F;
            float scaleY = (float)2.291667 * 3; //Ydensity / 96F;

            SKRect ViewBox = new SKRect(viewboxLeft_xps * scaleX, viewboxTop_xps * scaleY, (viewboxLeft_xps + viewboxWidth_xps) * scaleX, (viewboxTop_xps + viewboxHeight_xps) * scaleY);
            SKRect ViewPort = new SKRect(viewportLeft_xps, viewportTop_xps, viewportLeft_xps + viewportWidth_xps, viewportTop_xps + viewportHeight_xps);


            Stream xps_image_stream = zip_file.GetInputStream(zip_file.GetEntry(this.ImageSource.Substring(1)));
            var bmp = SKBitmap.Decode(xps_image_stream);



            //SKRect cut_view_box = new SKRect(Viewbox.Left, Viewbox.Top, Viewbox.Width / 3, Viewbox.Height / 3);
            SKRectI bitmapBox = SKRectI.Round(ViewBox);
            var bmp0 = new SKBitmap(bitmapBox.Width, bitmapBox.Height);

            Console.WriteLine("=============SKBitmap1=================");
            Console.WriteLine(scaleX);
            Console.WriteLine(scaleY);
            Console.WriteLine(this.Viewbox);
            Console.WriteLine(this.Viewport);
            Console.WriteLine(bmp.Width);
            Console.WriteLine(bmp.Height);
            Console.WriteLine(bitmapBox.Width);
            Console.WriteLine(bitmapBox.Height);

            bmp.ExtractSubset(bmp0, bitmapBox);

            string[] matrix_number = text_matrix.Split(",");
            SKMatrix LocalMatrix = new SKMatrix(float.Parse(matrix_number[0]), float.Parse(matrix_number[2]), float.Parse(matrix_number[4]), float.Parse(matrix_number[1]), float.Parse(matrix_number[3]), float.Parse(matrix_number[5]), 0, 0, 1);

            SKMatrix PreScale = SKMatrix.CreateScale(viewportWidth_xps / viewboxWidth_xps, viewportHeight_xps / viewboxHeight_xps);
            LocalMatrix = LocalMatrix.PreConcat(PreScale);

            SKPictureRecorder record = new SKPictureRecorder();
            SKCanvas recording_anvas = record.BeginRecording(ViewBox);
            recording_anvas.DrawBitmap(bmp0, 0, 0);
            SKPicture picture = record.EndRecording();

            //this.InternalPaint.Shader = SKShader.CreateBitmap(bmp0, this.tile_mode_x, this.tile_mode_y, LocalMatrix);
            this.InternalPaint.Shader = SKShader.CreatePicture(picture, this.tile_mode_x, this.tile_mode_y, LocalMatrix, ViewBox);
        }

        //public void Draw(SKCanvas canvas)
        //{
        //    canvas.DrawPath(SKPath.ParseSvgPathData("M 90,73.2 L 194.25,73.2 L 194.25,133.2 L 90,133.2 Z"), this.InternalPaint);
        //}

    }


    public class XPSVisualBrush_for_parse : XPSBrush_for_parse
    {
        public XPSVisualBrush_Data xps_visual_brush_data = new XPSVisualBrush_Data();
        public SKPaint InternalPaint = new SKPaint();

        public bool AutoLayoutContent { get; set; }
        public XPSVisual Visual { get; set; }

        SKRect Viewbox;
        SKRect Viewport;

        public SKShaderTileMode tile_mode_x;
        public SKShaderTileMode tile_mode_y;
        public string Data;
        public XPSBrush linear_gradient_brush;
        public SKPath path;
        public string RenderTransform;
        public XPSPath_for_parse xps_path;
        public XPSGlyphs xps_glyphs;
        public string glyphs_signal;

        public XPSVisualBrush_for_parse(XmlReader Source) : base(Source)
        {

        }

        public XPSVisualBrush_Data parse_VisualBrush(XmlReader source, Java.Util.Zip.ZipFile zip_file, SKCanvas canvas)
        {
            
            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element)
            {
                if (source.Name == "VisualBrush" && source.NodeType == XmlNodeType.Element)
                {
                    Console.WriteLine("<" + source.Name + ">");
                    Console.WriteLine(source.GetAttribute("Viewbox"));
                    Console.WriteLine(source.GetAttribute("TileMode"));
                    Console.WriteLine(source.GetAttribute("ViewboxUnits"));
                    Console.WriteLine(source.GetAttribute("ViewportUnits"));
                    Console.WriteLine(source.GetAttribute("Viewport"));

                    this.xps_visual_brush_data.Viewbox = source.GetAttribute("Viewbox");
                    this.xps_visual_brush_data.TileMode = source.GetAttribute("TileMode");
                    this.xps_visual_brush_data.ViewboxUnits = source.GetAttribute("ViewboxUnits");
                    this.xps_visual_brush_data.ViewportUnits = source.GetAttribute("ViewportUnits");
                    this.xps_visual_brush_data.Viewport = source.GetAttribute("Viewport");

                }
            }


            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "Path" && source.NodeType == XmlNodeType.Element)
                    {
                        XPSPath_for_parse xps_path_for_parse = new XPSPath_for_parse(source);
                        this.xps_visual_brush_data.Visual = xps_path_for_parse.parse_Path(source,zip_file,canvas);
                        xps_path = xps_path_for_parse;
                    }

                    if (source.Name == "VisualBrush" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        jump_end_element(source);
                        break;
                    }
                }
            }

            OnParsed(source);
            return xps_visual_brush_data;
        }


        private void OnParsed(XmlReader source)
        {
            this.InternalPaint.Style = SKPaintStyle.Fill;


            string[] viewboxContent = this.xps_visual_brush_data.Viewbox.Split(",");
            float viewboxLeft = float.Parse(viewboxContent[0]);
            float viewboxTop = float.Parse(viewboxContent[1]);
            float viewboxWidth = float.Parse(viewboxContent[2]);
            float viewboxHeight = float.Parse(viewboxContent[3]);

            //SKRect is the left top right bottom
            this.Viewbox = new SKRect((int)viewboxLeft, (int)viewboxTop, (int)(viewboxLeft + viewboxWidth), (int)(viewboxTop + viewboxHeight));

            string[] viewportContent = this.xps_visual_brush_data.Viewport.Split(",");
            float viewportLeft = float.Parse(viewportContent[0]);
            float viewportTop = float.Parse(viewportContent[1]);
            float viewportWidth = float.Parse(viewportContent[2]);
            float viewportHeight = float.Parse(viewportContent[3]);

            this.Viewport = new SKRect(viewportLeft, viewportTop, (viewportLeft + viewportWidth), (viewportTop + viewportHeight));

            string tile_mode_string = this.xps_visual_brush_data.TileMode;

            if (tile_mode_string == "None")
            {
                this.tile_mode_x = SKShaderTileMode.Decal;
                this.tile_mode_y = SKShaderTileMode.Decal;
            }
            else if (tile_mode_string == "FlipX")
            {
                this.tile_mode_x = SKShaderTileMode.Mirror;
                this.tile_mode_y = SKShaderTileMode.Decal;
            }
            else if (tile_mode_string == "FlipY")
            {
                this.tile_mode_x = SKShaderTileMode.Decal;
                this.tile_mode_y = SKShaderTileMode.Mirror;
            }
            else if (tile_mode_string == "FlipXY")
            {
                this.tile_mode_x = SKShaderTileMode.Mirror;
                this.tile_mode_y = SKShaderTileMode.Mirror;
            }
            else if (tile_mode_string == "Tile")
            {
                this.tile_mode_x = SKShaderTileMode.Repeat;
                this.tile_mode_y = SKShaderTileMode.Repeat;
            }



            float viewboxLeft_xps = this.Viewbox.Left;
            float viewboxTop_xps = this.Viewbox.Top;
            float viewboxWidth_xps = this.Viewbox.Width;
            float viewboxHeight_xps = this.Viewbox.Height;

            float viewportLeft_xps = this.Viewport.Left;
            float viewportTop_xps = this.Viewport.Top;
            float viewportWidth_xps = this.Viewport.Width;
            float viewportHeight_xps = this.Viewport.Height;



            SKRect ViewBox = new SKRect(viewboxLeft_xps, viewboxTop_xps, (viewboxLeft_xps + viewboxWidth_xps), (viewboxTop_xps + viewboxHeight_xps));
            SKMatrix pre_scale_matrix = SKMatrix.CreateScale(viewportWidth_xps / viewboxWidth_xps, viewportHeight_xps / viewboxHeight_xps);


            SKPictureRecorder record = new SKPictureRecorder();
            SKCanvas recording_anvas = record.BeginRecording(ViewBox);

            if (glyphs_signal == "Glyphs")
            {
                xps_glyphs.Draw(recording_anvas);
            }
            else
            {
                xps_path.Draw(recording_anvas);
            }
            //recording_anvas.SetMatrix(pre_matrix);
            SKPicture picture = record.EndRecording();

            this.InternalPaint.Shader = SKShader.CreatePicture(picture, this.tile_mode_x, this.tile_mode_x, pre_scale_matrix, ViewBox);
        }

    }

    public class XPSLinearGradientBrush_for_parse : XPSBrush_for_parse
    {
        public XPSLinearGradientBrush_Data xps_linear_gradient_brush_data = new XPSLinearGradientBrush_Data();
        public SKPaint InternalPaint = new SKPaint();

        public string MappingMode;
        public SKPoint StartPoint { get; set; }
        public SKPoint EndPoint { get; set; }

        public SKShaderTileMode TileMode;

        public string SpreadMethod;


        public XPSLinearGradientBrush_for_parse(XmlReader Source) : base(Source)
        {

        }

        public XPSLinearGradientBrush_Data parse_LinearGradientBrush(XmlReader source)
        {

            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element)
            {
                if (source.Name == "LinearGradientBrush" && source.NodeType == XmlNodeType.Element)
                {
                    Console.WriteLine("<" + source.Name + ">");
                    Console.WriteLine(source.GetAttribute("MappingMode"));
                    Console.WriteLine(source.GetAttribute("StartPoint"));
                    Console.WriteLine(source.GetAttribute("EndPoint"));
                    Console.WriteLine(source.GetAttribute("SpreadMethod"));

                    this.xps_linear_gradient_brush_data.MappingMode = source.GetAttribute("MappingMode");
                    this.xps_linear_gradient_brush_data.StartPoint = source.GetAttribute("StartPoint");
                    this.xps_linear_gradient_brush_data.EndPoint = source.GetAttribute("EndPoint");
                    this.xps_linear_gradient_brush_data.SpreadMethod = source.GetAttribute("SpreadMethod");
                }
            }

            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "LinearGradientBrush.GradientStops" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine("< GradientStop Color = #FF0000 Offset = 0.0 /> ");
                        XPSGradientStop_for_parse xps_gradient_stop_parse = new XPSGradientStop_for_parse(source);
                        xps_gradient_stop_parse.parse_GradientStop(source, source.Name);
                        this.xps_linear_gradient_brush_data.GradientStops = xps_gradient_stop_parse;

                    }

                    if (source.Name == "LinearGradientBrush" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        jump_end_element(source);
                        break;
                    }
                }
            }
            Parsed(source);
            return this.xps_linear_gradient_brush_data;
        }


        private void Parsed(XmlReader source)
        {
            this.MappingMode = this.xps_linear_gradient_brush_data.MappingMode;
            this.StartPoint = new SKPoint(float.Parse(this.xps_linear_gradient_brush_data.StartPoint.Split(",")[0]), float.Parse(this.xps_linear_gradient_brush_data.StartPoint.Split(",")[1]));
            this.EndPoint = new SKPoint(float.Parse(this.xps_linear_gradient_brush_data.EndPoint.Split(",")[0]), float.Parse(this.xps_linear_gradient_brush_data.EndPoint.Split(",")[1]));
            this.SpreadMethod = this.xps_linear_gradient_brush_data.SpreadMethod;


            if (this.SpreadMethod == "Pad")
            {
                this.TileMode = SKShaderTileMode.Clamp;
            }
            else if (this.SpreadMethod == "Repeat")
            {
                this.TileMode = SKShaderTileMode.Repeat;
            }
            else if (this.SpreadMethod == "Reflect")
            {
                this.TileMode = SKShaderTileMode.Mirror;
            }



            if (this.SpreadMethod == "Pad")
            {
                this.TileMode = SKShaderTileMode.Clamp;
            }
            else if (this.SpreadMethod == "Repeat")
            {
                this.TileMode = SKShaderTileMode.Repeat;
            }
            else if (this.SpreadMethod == "Reflect")
            {
                this.TileMode = SKShaderTileMode.Mirror;
            }

            List<SKColor> color_list = new List<SKColor>();
            List<float> offset_list = new List<float>();

            for (int i = 0; i < this.xps_linear_gradient_brush_data.GradientStops.xps_gradient_stop_data_list.Count; i++)
            {
                if (this.xps_linear_gradient_brush_data.GradientStops.xps_gradient_stop_data_list[i].Color.Length == 7)
                {
                    color_list.Add(new SKColor(uint.Parse("FF" + this.xps_linear_gradient_brush_data.GradientStops.xps_gradient_stop_data_list[i].Color.Substring(1), System.Globalization.NumberStyles.HexNumber)));
                }
                else if (this.xps_linear_gradient_brush_data.GradientStops.xps_gradient_stop_data_list[i].Color.Length == 9)
                {
                    color_list.Add(new SKColor(uint.Parse(this.xps_linear_gradient_brush_data.GradientStops.xps_gradient_stop_data_list[i].Color.Substring(1), System.Globalization.NumberStyles.HexNumber)));
                }

                offset_list.Add(float.Parse(this.xps_linear_gradient_brush_data.GradientStops.xps_gradient_stop_data_list[i].Offset));
            }

            this.InternalPaint.Shader = SKShader.CreateLinearGradient(this.StartPoint, this.EndPoint, color_list.ToArray(), offset_list.ToArray(), this.TileMode);
        }
    }


    public class XPSRadialGradientBrush_for_parse : XPSBrush_for_parse
    {
        public SKPaint InternalPaint = new SKPaint();

        public XPSRadialGradientBrush_Data xps_radial_gradient_brush_data = new XPSRadialGradientBrush_Data();

        public string MappingMode;
        public SKPoint Center { get; set; }
        public SKPoint GradientOrigin { get; set; }
        public float RadiusX { get; set; }
        public float RadiusY { get; set; }

        public SKShaderTileMode TileMode;

        public string SpreadMethod;

        public XPSRadialGradientBrush_for_parse(XmlReader Source) : base(Source)
        {

        }

        public XPSRadialGradientBrush_Data parse_RadialGradientBrush(XmlReader source)
        {

            
            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element)
            {
                if (source.Name == "RadialGradientBrush" && source.NodeType == XmlNodeType.Element)
                {
                    Console.WriteLine("<" + source.Name + ">");
                    Console.WriteLine(source.GetAttribute("MappingMode"));
                    Console.WriteLine(source.GetAttribute("Center"));
                    Console.WriteLine(source.GetAttribute("GradientOrigin"));
                    Console.WriteLine(source.GetAttribute("RadiusX"));
                    Console.WriteLine(source.GetAttribute("RadiusY"));
                    Console.WriteLine(source.GetAttribute("SpreadMethod"));

                    this.xps_radial_gradient_brush_data.MappingMode = source.GetAttribute("MappingMode");
                    this.xps_radial_gradient_brush_data.Center = source.GetAttribute("Center");
                    this.xps_radial_gradient_brush_data.GradientOrigin = source.GetAttribute("GradientOrigin");
                    this.xps_radial_gradient_brush_data.RadiusX = source.GetAttribute("RadiusX");
                    this.xps_radial_gradient_brush_data.RadiusY = source.GetAttribute("RadiusY");
                    this.xps_radial_gradient_brush_data.SpreadMethod = source.GetAttribute("SpreadMethod");

                }
            }


            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "RadialGradientBrush.GradientStops" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine("< GradientStop Color = #FF0000 Offset = 0.0 /> ");
                        XPSGradientStop_for_parse xps_gradient_stop_parse = new XPSGradientStop_for_parse(source);
                        xps_gradient_stop_parse.parse_GradientStop(source, source.Name);
                        this.xps_radial_gradient_brush_data.GradientStops = xps_gradient_stop_parse;
                    }

                    if (source.Name == "RadialGradientBrush" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                        jump_end_element(source);
                        break;
                    }
                }
            }

            Parsed(source);

            return this.xps_radial_gradient_brush_data;
        }


        private void Parsed(XmlReader source)
        {
            this.MappingMode = this.xps_radial_gradient_brush_data.MappingMode;
            this.Center = new SKPoint(float.Parse( this.xps_radial_gradient_brush_data.Center.Split(",")[0] ), float.Parse( this.xps_radial_gradient_brush_data.Center.Split(",")[1] ));
            this.GradientOrigin = new SKPoint(float.Parse(this.xps_radial_gradient_brush_data.GradientOrigin.Split(",")[0]), float.Parse(this.xps_radial_gradient_brush_data.GradientOrigin.Split(",")[1]));
            this.RadiusX = float.Parse(this.xps_radial_gradient_brush_data.RadiusX);
            this.RadiusY = float.Parse(this.xps_radial_gradient_brush_data.RadiusY);
            this.SpreadMethod = this.xps_radial_gradient_brush_data.SpreadMethod;


            if (this.SpreadMethod == "Pad")
            {
                this.TileMode = SKShaderTileMode.Clamp;
            }
            else if (this.SpreadMethod == "Repeat")
            {
                this.TileMode = SKShaderTileMode.Repeat;
            }
            else if (this.SpreadMethod == "Reflect")
            {
                this.TileMode = SKShaderTileMode.Mirror;
            }


            List<SKColor> color_list = new List<SKColor>();
            List<float> offset_list = new List<float>();

            for (int i = 0 ; i < this.xps_radial_gradient_brush_data.GradientStops.xps_gradient_stop_data_list.Count; i++)
            {
                if (this.xps_radial_gradient_brush_data.GradientStops.xps_gradient_stop_data_list[i].Color.Length == 7)
                {
                    color_list.Add( new SKColor( uint.Parse( "FF" + this.xps_radial_gradient_brush_data.GradientStops.xps_gradient_stop_data_list[i].Color.Substring(1), System.Globalization.NumberStyles.HexNumber ) ) );
                }
                else if (this.xps_radial_gradient_brush_data.GradientStops.xps_gradient_stop_data_list[i].Color.Length == 9)
                {
                    color_list.Add( new SKColor( uint.Parse( this.xps_radial_gradient_brush_data.GradientStops.xps_gradient_stop_data_list[i].Color.Substring(1), System.Globalization.NumberStyles.HexNumber ) ) );
                }

                offset_list.Add( float.Parse( this.xps_radial_gradient_brush_data.GradientStops.xps_gradient_stop_data_list[i].Offset ) );
            }

            SKMatrix gradient_matrix = new SKMatrix();
            float ScaleY = this.RadiusY / this.RadiusX;
            gradient_matrix = new SKMatrix(1, 0, 0, 0, ScaleY, -(ScaleY - 1) * Center.Y, 0, 0, 1);

            this.InternalPaint.Shader = SKShader.CreateTwoPointConicalGradient(this.GradientOrigin, 0, this.Center, this.RadiusX, color_list.ToArray(), offset_list.ToArray(), this.TileMode, gradient_matrix);
        }
    }

    public class XPSGradientStop_for_parse : XPSParsable
    {
        public List<XPSGradientStop_Data> xps_gradient_stop_data_list = new List<XPSGradientStop_Data>(0);

        public XPSGradientStop_for_parse(XmlReader Source) : base(Source)
        {

        }

        public List<XPSGradientStop_Data> parse_GradientStop(XmlReader source, string brush_name)
        {
            

            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element)
            {
                if (source.Name == brush_name && source.NodeType == XmlNodeType.Element)
                {
                    Console.WriteLine("<" + source.Name + ">");
                }
            }



            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "GradientStop" && source.IsEmptyElement)
                    {
                        Console.WriteLine("< GradientStop/> ");
                        Console.WriteLine(source.GetAttribute("Color"));
                        Console.WriteLine(source.GetAttribute("Offset"));
                        XPSGradientStop_Data xps_gradientStop_data = new XPSGradientStop_Data();
                        xps_gradientStop_data.Color = source.GetAttribute("Color");
                        xps_gradientStop_data.Offset = source.GetAttribute("Offset");

                        this.xps_gradient_stop_data_list.Add(xps_gradientStop_data);
                    }

                    if (source.Name == brush_name && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");

                        jump_end_element(source);
                        break;
                    }
                }
            }
            return this.xps_gradient_stop_data_list;
        }
    }


    public abstract class XPSPanel : XPSVisual_for_parse
    {
        protected readonly List<XPSVisual_for_parse> Children = new List<XPSVisual_for_parse>();

        public XPSPanel(XmlReader Source) : base(Source) { }
    }


    public class XPSPathGeometry_for_parse : XPSParsable
    {

        public List<XPSPathFigure> Figures = new List<XPSPathFigure>(0);
        public string FillRule { get; set; }

        private SKPathFillType InternalFillRule;
        private readonly SKPath InternalData = new SKPath();
        public XPSPathGeometry_Data xps_path_geometry_data = new XPSPathGeometry_Data();

        public string signal;
        

        public XPSPathGeometry_for_parse( XmlReader source) : base(source)
        {

        }

        public XPSPathGeometry_Data parse_PathGeometry(XmlReader source)
        {

            //read the object GetAttribute
            if (source.NodeType == XmlNodeType.Element)
            {
                if (source.Name == "Path.Data" && source.NodeType == XmlNodeType.Element)
                {
                    Console.WriteLine("<" + source.Name + ">");
                }
            }



            //read the object Elements
            while (source.Read())
            {
                if (source.NodeType == XmlNodeType.Element || source.NodeType == XmlNodeType.Text || source.NodeType == XmlNodeType.EndElement)
                {
                    if (source.Name == "PathGeometry" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine("<" + source.Name + ">");
                        Console.WriteLine(source.GetAttribute("Transform"));

                        if(source.GetAttribute("Transform")!=null)
                        {
                            xps_path_geometry_data.Transform = source.GetAttribute("Transform");
                        }
                        
                    }

                    if (source.Name == "PathFigure" && source.NodeType == XmlNodeType.Element)
                    {
                        Console.WriteLine("<" + source.Name + ">");
                        XPSPathFigure xps_path_figure = new XPSPathFigure();
                        xps_path_figure.parse_PathFigure(source);
                        xps_path_figure.BuildInternalData();
                        Figures.Add(xps_path_figure);

                    }

                    if (source.Name == "PathGeometry" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");
                    }

                    if (source.Name == "Path.Data" && source.NodeType == XmlNodeType.EndElement)
                    {
                        Console.WriteLine("</" + source.Name + ">");

                        jump_end_element(source);
                        break;
                    }
                }
            }

            return this.xps_path_geometry_data;

        }

    }



    public class XPS_Data
    { 
    
    }

    public class XPSFixedPage_Data : XPS_Data
    {
        public string xmlns;
        public string Width;
        public string Height;
        public string xml_lang;
        public List<XPS_Data> xps_data_children = new List<XPS_Data>();
    }

    public class XPSCanvas_Data : XPS_Data
    {
        public string Clip;
        public string RenderTransform;
        public List<XPS_Data> xps_data_children = new List<XPS_Data>();
    }

    public class XPSPath_Data : XPS_Data
    {
        public string Data;
        public string Stroke;
        public string StrokeThickness;
        public XPSBrush_for_parse Fill;
        public string OpacityMusk;
        public string Clip;
        public string RenderTransform;
    }

    public class XPSGlyphs_Data : XPS_Data
    {
        public string Name;
        public string BidiLevel;
        public string Fill;
        public string FontUri;
        public string FontRenderingEmSize;
        public string StyleSimulations;
        public string OriginX;
        public string OriginY;
        public string UnicodeString;
        public string Indices;
        public string xml_lang;
    }

    public class XPSBrush_Data : XPS_Data
    {
        public SKPaint InternalPaint = new SKPaint();

        public string signal;

        public double Opacity { get; set; }
        public string RelativeTransform { get; set; } // public abstract class Transform : System.Windows.Media.GeneralTransform
        public string Transform { get; set; }         // public abstract class Transform : System.Windows.Media.GeneralTransform

        public SKPaint ToPaint()
        {
            return InternalPaint;
        }

    }

    public class XPSImageBrush_Data : XPSBrush_Data
    {
        public string ImageSource;
        public string Viewbox;
        public string TileMode;
        public string ViewboxUnits;
        public string ViewportUnits;
        public string Viewport;
    }

    public class XPSVisualBrush_Data : XPSBrush_Data
    {
        public string Viewbox;
        public string TileMode;
        public string ViewboxUnits;
        public string ViewportUnits;
        public string Viewport;
        public XPSPath_Data Visual;
    }


    public class XPSLinearGradientBrush_Data : XPSBrush_Data
    {
        public string MappingMode;
        public string StartPoint;
        public string EndPoint;
        public string SpreadMethod;
        public XPSGradientStop_for_parse GradientStops = new XPSGradientStop_for_parse(null);
    }

    public class XPSRadialGradientBrush_Data : XPSBrush_Data
    {
        public string MappingMode;
        public string Center;
        public string GradientOrigin;
        public string RadiusX;
        public string RadiusY;
        public string SpreadMethod;
        public XPSGradientStop_for_parse GradientStops = new XPSGradientStop_for_parse(null);
    }


    public class XPSGradientStop_Data : XPS_Data
    {
        public string Color;
        public string Offset;
    }

    public class XPSPathGeometry_Data : XPS_Data
    {
        public string Transform;
    }



    //*/
}

