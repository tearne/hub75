use std::env;

fn main() {
    // OWNER_TAG bakes a free-form owner/contact string into the USB product descriptor
    // at build time: `hub75 (<tag>)` when set, or plain `hub75` when unset/blank. The
    // string is composed here rather than with env!/concat! in source because the unset
    // case needs a fallback (env! would fail to compile) and no_std firmware can't build
    // the string at runtime.
    let product = match env::var("OWNER_TAG") {
        Ok(tag) if !tag.trim().is_empty() => format!("hub75 ({tag})"),
        _ => "hub75".to_string(),
    };
    println!("cargo:rustc-env=USB_PRODUCT_STRING={product}");
    println!("cargo:rerun-if-env-changed=OWNER_TAG");
}
