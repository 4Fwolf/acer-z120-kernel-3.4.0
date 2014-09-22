#!perl -w
my $prj = $ARGV[0];
my $outdir = "out/target/product/$prj";

if($#ARGV == 0) {
    &add_magic();
} else {
    &usage();
}

sub add_magic {
    my $us_offset = 0x100;
    my $bs_offset = 0x100;
    my $ss_offset = 0x100;
    my $pattern = "109f10eed3f021e3";

    if (-e "$outdir/uboot_$prj.bin") {
        open FP, "+<$outdir/uboot_$prj.bin" or die "can't open uboot image!\n";
        binmode FP;
        seek FP, $us_offset, 0 or die $!;
        print FP $pattern;
        print "uboot_$prj.bin signed.\n";
        close FP;
    }

    if (-e "$outdir/boot.img") {
        open FP, "+<$outdir/boot.img" or die "can't open boot image!\n";
        binmode FP;
        seek FP, $bs_offset, 0 or die $!;
        print FP $pattern;
        print "boot.img signed.\n";
        close FP;
    }

    if (-e "$outdir/system.img") {
        open FP, "+<$outdir/system.img" or die "can't open system image!\n";
        binmode FP;
        seek FP, $ss_offset, 0 or die $!;
        print FP $pattern;
        print "system.img signed.\n";
        close FP;
    }
}

sub usage {
    print "usage: perl jrd_magic.pl project_name\)n";
    exit (0);
}
